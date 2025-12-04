// SPDX-License-Identifier: GPL-2.0
// Razer PWM PC Fan Controller (USB HID) -> Linux hwmon driver
//
// Design goals
// - Expose fan RPM and PWM control through the standard Linux hwmon sysfs ABI so
//   normal user tools work out of the box: lm-sensors, sensors, pwmconfig, fancontrol.
// - Talk to the device using HID Feature reports (SET_REPORT / GET_REPORT).
//
// Protocol notes (based on public implementations and observed behavior)
// - Feature report packet length is 91 bytes.
// - Device may reply with BUSY status and requires polling GET_REPORT until SUCCESS.
// - Sequence number increments by 0x08 and must never be 0x00.
//
// hwmon interface exported by this driver
// - fan1_input .. fan8_input: measured RPM
// - pwm1 .. pwm8: PWM level in hwmon convention range 0..255
// - pwm1_enable .. pwm8_enable: control mode indicator
//     0 = force full speed (we implement as 100 percent)
//     1 = manual mode (normal mode for this device protocol)
//     2 = default "auto-like" (we implement as 50 percent)
//
// Important limitation
// - Public protocol examples only show a manual mode value (0x04). This driver does not
//   implement a real hardware "automatic" mode; pwm_enable=2 is a user-friendly preset.

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#define DRIVER_NAME "razer_pwm_hwmon"

// Razer USB IDs for this controller
#define USB_VENDOR_ID_RAZER 0x1532
#define USB_DEVICE_ID_RAZER_PWM_FAN_CONTROLLER 0x0f3c

// The controller exposes 8 independent PWM/tach channels
#define CHANNEL_COUNT 8

// Timing used by the Windows-side implementations:
// - after SET_REPORT, wait a small delay before first GET_REPORT
// - if status is BUSY, keep polling with a short sleep between polls
#define DEVICE_READ_DELAY_MS    5
#define DEVICE_READ_TIMEOUT_MS  500

// Default speed preset used by many implementations
#define DEFAULT_SPEED_PERCENT 50

// Packet layout constants
// - Full feature report is 91 bytes
// - Data payload area inside the packet is 80 bytes
#define PKT_LEN 91
#define DATA_LEN 80

// Device status values found in responses
#define ST_DEFAULT 0x00
#define ST_BUSY    0x01
#define ST_SUCCESS 0x02
#define ST_ERROR   0x03
#define ST_TIMEOUT 0x04
#define ST_INVALID 0x05

// Command classes
#define CC_INFO 0x00
#define CC_PWM  0x0d

// PWM commands
#define CMD_SET_CHANNEL_PERCENT 0x0d
#define CMD_SET_CHANNEL_MODE    0x02
#define CMD_GET_CHANNEL_SPEED   0x81

// Channel addressing inside the data payload
// - Data[0] is a constant group selector byte (0x01 in the known protocol)
// - Data[1] identifies which port/channel is being addressed: 0x05 + channel_index
#define CH_BASE  0x05
#define CH_GROUP 0x01

// Device instance state
// - lock serializes USB transactions + protects cached state
// - seq keeps protocol sequence number (increments by 0x08, never 0)
// - pwm_percent holds last applied percent value (0..100) per channel
// - pwm_enable holds last requested enable/mode value (0/1/2) per channel
// - rpm_cache and rpm_cache_jiffies implement a simple read cache to reduce USB traffic
struct razer_pwm_dev {
	struct hid_device *hdev;
	struct device *hwmon_dev;

	struct mutex lock;

	u8 seq;

	u8 pwm_percent[CHANNEL_COUNT];
	u8 pwm_enable[CHANNEL_COUNT];

	u16 rpm_cache[CHANNEL_COUNT];
	unsigned long rpm_cache_jiffies;
};

// Compute packet checksum.
// The protocol uses XOR of bytes 3..88 inclusive.
// This checksum is placed into byte 89 of the outgoing packet.
static u8 razer_checksum(const u8 *buf)
{
	u8 r = 0;
	int i;

	for (i = 3; i < 89; i++)
		r ^= buf[i];

	return r;
}

// Advance the protocol sequence number.
// The known device logic expects sequence numbers in steps of 0x08 and never 0x00.
static u8 razer_next_seq(struct razer_pwm_dev *d)
{
	do {
		d->seq += 0x08;
	} while (d->seq == 0x00);

	return d->seq;
}

// Send a HID Feature report (SET_REPORT).
// Arguments:
// - report_id is the HID report ID; this device uses 0 for "unnumbered" reports.
// - buf points to the full packet (PKT_LEN bytes).
// Return value of hid_hw_raw_request is usually number of bytes transferred or negative errno.
// We pass it through for the caller to interpret.
static int razer_feature_set(struct hid_device *hdev, u8 report_id, u8 *buf, size_t len)
{
	return hid_hw_raw_request(hdev, report_id, buf, len, HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
}

// Read a HID Feature report (GET_REPORT).
// Many HID paths want buf[0] to contain the report ID requested.
// For report_id=0, that means buf[0]=0.
// Return value is bytes transferred or negative errno.
static int razer_feature_get(struct hid_device *hdev, u8 report_id, u8 *buf, size_t len)
{
	buf[0] = report_id;
	return hid_hw_raw_request(hdev, report_id, buf, len, HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
}

// Write a packet and read its response.
// Behavior:
// - SET_REPORT the outgoing 91-byte packet
// - wait a short delay so the device can process it
// - GET_REPORT a response packet
// - if response status is BUSY, keep polling GET_REPORT until SUCCESS or timeout
// - additionally check that response sequence matches request sequence
//
// Error handling policy:
// - Any negative return from hid_hw_raw_request is an immediate error.
// - If we keep seeing BUSY until timeout, return -ETIMEDOUT.
// - If we receive a non-SUCCESS final status, return -EIO.
// - If sequence mismatches (stale response), we attempt a few quick reads to drain it.
static int razer_write_and_read(struct razer_pwm_dev *d, const u8 *tx, u8 *rx)
{
	int ret;
	unsigned long timeout;

	ret = razer_feature_set(d->hdev, tx[0], (u8 *)tx, PKT_LEN);
	if (ret < 0)
		return ret;

	msleep(DEVICE_READ_DELAY_MS);

	timeout = jiffies + msecs_to_jiffies(DEVICE_READ_TIMEOUT_MS);

	do {
		ret = razer_feature_get(d->hdev, tx[0], rx, PKT_LEN);
		if (ret < 0)
			return ret;

		// Some HID stacks could theoretically return a shorter read.
		// We require at least the header bytes [0..8] to be present.
		if (ret < 9)
			return -EIO;

		// BUSY means the device hasn't finished processing the request yet.
		// Poll until it becomes SUCCESS or timeout hits.
		if (rx[1] == ST_BUSY) {
			msleep(DEVICE_READ_DELAY_MS);
			continue;
		}

		break;

	} while (time_before(jiffies, timeout));

	if (rx[1] == ST_BUSY)
		return -ETIMEDOUT;

	if (rx[1] != ST_SUCCESS)
		return -EIO;

	// Sequence mismatch can happen if a previous response is still in the buffer.
	// We try to read a few more times to obtain the response with matching sequence.
	if (rx[2] != tx[2]) {
		int tries;

		for (tries = 0; tries < 3; tries++) {
			msleep(DEVICE_READ_DELAY_MS);

			ret = razer_feature_get(d->hdev, tx[0], rx, PKT_LEN);
			if (ret < 0)
				return ret;

			if (ret >= 9 && rx[1] == ST_SUCCESS && rx[2] == tx[2])
				return 0;
		}

		return -EIO;
	}

	return 0;
}

// Build an outgoing protocol packet in a simple flat buffer.
// Fields:
// - [0] ReportId: 0 for this device
// - [1] Status: ST_DEFAULT for requests
// - [2] SequenceNumber: caller-supplied
// - [3..4] RemainingCount: 0
// - [5] ProtocolType: 0
// - [6] DataLength: caller-supplied
// - [7] CommandClass
// - [8] Command
// - [9..] payload data (up to DataLength bytes); remaining bytes are zero
// - [89] checksum XOR of bytes 3..88
// - [90] reserved: 0
//
// Note: We always zero the entire packet, so checksum is deterministic.
static void razer_build_packet(u8 *buf,
			       u8 seq,
			       u8 data_len,
			       u8 cmd_class,
			       u8 cmd,
			       const u8 *data)
{
	memset(buf, 0, PKT_LEN);

	buf[0] = 0x00;
	buf[1] = ST_DEFAULT;
	buf[2] = seq;

	buf[3] = 0x00;
	buf[4] = 0x00;

	buf[5] = 0x00;

	buf[6] = data_len;
	buf[7] = cmd_class;
	buf[8] = cmd;

	if (data && data_len > 0)
		memcpy(&buf[9], data, data_len);

	buf[89] = razer_checksum(buf);
	buf[90] = 0x00;
}

// Switch a channel into manual mode.
// Observed manual value is 0x04.
// Many devices require manual mode before accepting percent writes.
static int razer_set_channel_mode_manual(struct razer_pwm_dev *d, int ch)
{
	u8 tx[PKT_LEN], rx[PKT_LEN];
	u8 data[3];

	data[0] = CH_GROUP;
	data[1] = (u8)(CH_BASE + ch);
	data[2] = 0x04;

	razer_build_packet(tx, razer_next_seq(d), 3, CC_PWM, CMD_SET_CHANNEL_MODE, data);

	return razer_write_and_read(d, tx, rx);
}

// Set a channel PWM value in percent (0..100).
// This function does not clamp input; callers should ensure 0..100.
static int razer_set_channel_percent(struct razer_pwm_dev *d, int ch, u8 percent)
{
	u8 tx[PKT_LEN], rx[PKT_LEN];
	u8 data[3];

	data[0] = CH_GROUP;
	data[1] = (u8)(CH_BASE + ch);
	data[2] = percent;

	razer_build_packet(tx, razer_next_seq(d), 3, CC_PWM, CMD_SET_CHANNEL_PERCENT, data);

	return razer_write_and_read(d, tx, rx);
}

// Read a channel tachometer speed in RPM.
// Protocol detail:
// - Request uses DataLength=6 even though only data[0..1] are required.
// - Response encodes RPM as big-endian at Data[4..5].
// Data[] begins at packet offset 9, so the bytes are at rx[9+4] and rx[9+5].
static int razer_get_channel_rpm(struct razer_pwm_dev *d, int ch, u16 *rpm)
{
	u8 tx[PKT_LEN], rx[PKT_LEN];
	u8 data[6];

	int ret;

	memset(data, 0, sizeof(data));
	data[0] = CH_GROUP;
	data[1] = (u8)(CH_BASE + ch);

	razer_build_packet(tx, razer_next_seq(d), 6, CC_PWM, CMD_GET_CHANNEL_SPEED, data);

	ret = razer_write_and_read(d, tx, rx);
	if (ret < 0)
		return ret;

	*rpm = (u16)((rx[9 + 4] << 8) | rx[9 + 5]);
	return 0;
}

// Refresh RPM cache for all channels.
// This is a batch operation: 8 sequential USB transactions.
// The caching policy reduces sysfs read overhead when userspace polls frequently.
static int razer_refresh_rpm_cache(struct razer_pwm_dev *d)
{
	int i, ret;
	u16 rpm;

	for (i = 0; i < CHANNEL_COUNT; i++) {
		ret = razer_get_channel_rpm(d, i, &rpm);
		if (ret < 0)
			return ret;
		d->rpm_cache[i] = rpm;
	}

	d->rpm_cache_jiffies = jiffies;
	return 0;
}

// hwmon visibility callback.
// Returns file permission bits for each attribute, controlling which sysfs files exist.
// - fan_input is read-only
// - pwm_input and pwm_enable are read-write
static umode_t razer_hwmon_is_visible(const void *data,
				      enum hwmon_sensor_types type,
				      u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input)
			return 0444;
		break;

	case hwmon_pwm:
		if (attr == hwmon_pwm_input || attr == hwmon_pwm_enable)
			return 0644;
		break;

	default:
		break;
	}

	return 0;
}

// hwmon read callback.
// Called when userspace reads attributes under /sys/class/hwmon/... for this device.
static int razer_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long *val)
{
	struct razer_pwm_dev *d = dev_get_drvdata(dev);
	int ret = 0;

	if (channel < 0 || channel >= CHANNEL_COUNT)
		return -EINVAL;

	mutex_lock(&d->lock);

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input) {
			ret = -EOPNOTSUPP;
			break;
		}

		// Simple read cache: refresh at most every 500ms.
		// This avoids 8 USB transactions for every "sensors" refresh tick.
		if (time_after(jiffies, d->rpm_cache_jiffies + msecs_to_jiffies(500))) {
			ret = razer_refresh_rpm_cache(d);
			if (ret < 0)
				break;
		}

		*val = d->rpm_cache[channel];
		break;

	case hwmon_pwm:
		if (attr == hwmon_pwm_input) {
			// Convert cached percent (0..100) to hwmon pwm scale (0..255).
			// Use rounding so values map more naturally for users.
			*val = (d->pwm_percent[channel] * 255L + 50) / 100;
		} else if (attr == hwmon_pwm_enable) {
			*val = d->pwm_enable[channel];
		} else {
			ret = -EOPNOTSUPP;
		}
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&d->lock);
	return ret;
}

// hwmon write callback.
// Called when userspace writes pwmN or pwmN_enable.
// We always serialize writes because each write triggers HID transactions.
static int razer_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long val)
{
	struct razer_pwm_dev *d = dev_get_drvdata(dev);
	int ret = 0;

	if (channel < 0 || channel >= CHANNEL_COUNT)
		return -EINVAL;

	mutex_lock(&d->lock);

	switch (type) {
	case hwmon_pwm:
		if (attr == hwmon_pwm_enable) {
			// hwmon convention used by many drivers:
			// - 0: disable PWM control, usually results in full speed
			// - 1: manual mode, pwmN controls duty
			// - 2+: hardware automatic mode when supported
			//
			// This device protocol (publicly known) supports manual (0x04).
			// We implement:
			// - 0: force 100 percent
			// - 1: manual mode active (leave duty as-is)
			// - 2: set a "default" duty (50 percent)
			if (val == 0) {
				ret = razer_set_channel_mode_manual(d, channel);
				if (ret < 0)
					break;

				ret = razer_set_channel_percent(d, channel, 100);
				if (ret < 0)
					break;

				d->pwm_percent[channel] = 100;
				d->pwm_enable[channel] = 0;

			} else if (val == 1) {
				ret = razer_set_channel_mode_manual(d, channel);
				if (ret < 0)
					break;

				d->pwm_enable[channel] = 1;

			} else if (val == 2) {
				ret = razer_set_channel_mode_manual(d, channel);
				if (ret < 0)
					break;

				ret = razer_set_channel_percent(d, channel, DEFAULT_SPEED_PERCENT);
				if (ret < 0)
					break;

				d->pwm_percent[channel] = DEFAULT_SPEED_PERCENT;
				d->pwm_enable[channel] = 2;

			} else {
				ret = -EINVAL;
			}

		} else if (attr == hwmon_pwm_input) {
			// Userspace writes pwmN in 0..255.
			// Convert to 0..100 percent with rounding.
			long v = val;
			u8 percent;

			if (v < 0)
				v = 0;
			if (v > 255)
				v = 255;

			percent = (u8)((v * 100 + 127) / 255);

			// If the channel is not in manual mode, switch it to manual first.
			// This ensures percent writes are accepted.
			if (d->pwm_enable[channel] != 1) {
				ret = razer_set_channel_mode_manual(d, channel);
				if (ret < 0)
					break;

				d->pwm_enable[channel] = 1;
			}

			ret = razer_set_channel_percent(d, channel, percent);
			if (ret < 0)
				break;

			d->pwm_percent[channel] = percent;

		} else {
			ret = -EOPNOTSUPP;
		}
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&d->lock);
	return ret;
}

// hwmon ops table: binds our sysfs behavior to the hwmon core.
static const struct hwmon_ops razer_hwmon_ops = {
	.is_visible = razer_hwmon_is_visible,
	.read = razer_hwmon_read,
	.write = razer_hwmon_write,
};

// Declare which channels and attributes exist.
// We describe 8 fan inputs and 8 pwm outputs, each pwm has input+enable.
static const struct hwmon_channel_info *razer_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan,
		HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT,
		HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT),

	HWMON_CHANNEL_INFO(pwm,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE),

	NULL
};

static const struct hwmon_chip_info razer_chip_info = {
	.ops = &razer_hwmon_ops,
	.info = razer_hwmon_info,
};

// Initialize the device into a known state.
// We set all channels to manual mode and 50 percent duty.
// This mirrors typical behavior in vendor tools and public reference implementations.
static int razer_pwm_init_device(struct razer_pwm_dev *d)
{
	int i, ret;

	for (i = 0; i < CHANNEL_COUNT; i++) {
		ret = razer_set_channel_mode_manual(d, i);
		if (ret < 0)
			return ret;

		ret = razer_set_channel_percent(d, i, DEFAULT_SPEED_PERCENT);
		if (ret < 0)
			return ret;

		d->pwm_percent[i] = DEFAULT_SPEED_PERCENT;
		d->pwm_enable[i] = 1;
	}

	d->rpm_cache_jiffies = 0;
	return 0;
}

// HID probe: called when the HID core matches this driver to the USB device.
// Responsibilities:
// - allocate and initialize per-device state
// - start HID hardware and open it for raw requests
// - program initial PWM state
// - register a hwmon device to expose sysfs nodes
static int razer_pwm_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct razer_pwm_dev *d;

	d = devm_kzalloc(&hdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->hdev = hdev;
	mutex_init(&d->lock);
	hid_set_drvdata(hdev, d);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "hid_parse failed: %d\n", ret);
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "hid_hw_start failed: %d\n", ret);
		return ret;
	}

	// hid_hw_open enables low-level I/O paths for raw requests.
	// Some HID stacks operate more reliably with explicit open/close.
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hid_hw_open failed: %d\n", ret);
		hid_hw_stop(hdev);
		return ret;
	}

	mutex_lock(&d->lock);
	ret = razer_pwm_init_device(d);
	mutex_unlock(&d->lock);

	if (ret) {
		hid_err(hdev, "device init failed: %d\n", ret);
		hid_hw_close(hdev);
		hid_hw_stop(hdev);
		return ret;
	}

	// Register hwmon device using device-managed API.
	// This creates /sys/class/hwmon/hwmonX and the pwm/fan files declared above.
	d->hwmon_dev = devm_hwmon_device_register_with_info(&hdev->dev,
							    "razer_pwm_pc_fan",
							    d,
							    &razer_chip_info,
							    NULL);
	if (IS_ERR(d->hwmon_dev)) {
		ret = PTR_ERR(d->hwmon_dev);
		hid_err(hdev, "hwmon register failed: %d\n", ret);
		hid_hw_close(hdev);
		hid_hw_stop(hdev);
		return ret;
	}

	hid_info(hdev, "Razer PWM PC Fan Controller hwmon loaded\n");
	return 0;
}

// HID remove: called when the device is unplugged or driver is unloaded.
// We close the HID device and stop the HID hardware layer.
// devm-managed hwmon registration is automatically cleaned up by the device core.
static void razer_pwm_remove(struct hid_device *hdev)
{
	hid_info(hdev, "Razer PWM PC Fan Controller hwmon removed\n");
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

// HID device ID table.
// This driver binds to the Razer PWM controller USB VID/PID.
static const struct hid_device_id razer_pwm_table[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_RAZER, USB_DEVICE_ID_RAZER_PWM_FAN_CONTROLLER) },
	{ }
};
MODULE_DEVICE_TABLE(hid, razer_pwm_table);

static struct hid_driver razer_pwm_driver = {
	.name = DRIVER_NAME,
	.id_table = razer_pwm_table,
	.probe = razer_pwm_probe,
	.remove = razer_pwm_remove,
};

module_hid_driver(razer_pwm_driver);

MODULE_AUTHOR("SlimRG");
MODULE_DESCRIPTION("Razer PWM PC Fan Controller hwmon driver");
MODULE_LICENSE("GPL");
