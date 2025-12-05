// SPDX-License-Identifier: GPL-2.0
// Razer PWM PC Fan Controller (USB HID, 1532:0F3C)
// Exposes RPM + PWM via hwmon sysfs (fanN_input, pwmN, pwmN_enable).
//
// Protocol is based on widely used userland implementations:
// - LibreHardwareMonitor's RazerFanController.cs :contentReference[oaicite:5]{index=5}
// - FanControl.Razer PwmFanControllerDevice.cs :contentReference[oaicite:6]{index=6}
//
// Key idea:
// We send/receive 91-byte HID FEATURE reports.
// Layout (matches Windows implementations):
//   [0]  ReportId (0)
//   [1]  Status
//   [2]  SequenceNumber
//   [3]  RemainingCount hi
//   [4]  RemainingCount lo
//   [5]  ProtocolType
//   [6]  DataLength
//   [7]  CommandClass
//   [8]  Command
//   [9..88] Data (80 bytes)
//   [89] CRC (xor of bytes 3..88)
//   [90] Reserved
//
// Notes:
// - Some reads respond BUSY; we poll until SUCCESS or timeout.
// - We implement manual mode only for pwmN_enable=1.
// - pwmN_enable=0 sets a safe default 50% (documented in repo README). :contentReference[oaicite:7]{index=7}

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>

#define DRIVER_NAME "razer_pwm_pc_fan_controller"

#define CHANNEL_COUNT 8

#define PKT_LEN 91
#define DATA_LEN 80

// Status codes (match userland enums)
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
#define CMD_SET_CHANNEL_MODE    0x02
#define CMD_SET_CHANNEL_PERCENT 0x0d
#define CMD_GET_CHANNEL_SPEED   0x81

// Info command (firmware)
#define CMD_GET_FIRMWARE 0x87

// Channel mode values (observed in userland)
#define MODE_MANUAL 0x04

// Timing
#define DEVICE_READ_DELAY_MS   5
#define DEVICE_READ_TIMEOUT_MS 500

// Default when disabling manual control through pwmN_enable=0
#define DEFAULT_SPEED_PERCENT 50

// ===================== Debug / tuning =====================
// Runtime debugging:
//  - Set module param debug=1 to enable verbose logs + hex dumps to dmesg/journal.
//  - Additionally (recommended) use dynamic debug to selectively enable dev_dbg()/pr_debug()
//    callsites without recompiling.
//
// Tuning:
//  - force_write_ms re-applies cached PWM values periodically (best-effort) to combat
//    occasional controller "drift/resets" observed in the vendor tools ecosystem.
//    Set to 0 to disable.
struct razer_pwm_dev; // forward declaration for debug helpers

static bool debug = false;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Enable verbose debug logging (incl. HID packet hex dumps)");

static int force_write_ms = 2500;
module_param(force_write_ms, int, 0644);
MODULE_PARM_DESC(force_write_ms, "Force re-apply cached PWM values every N ms (0=disable)");

#define razer_dev(d) (&(d)->hdev->dev)

#define razer_dbg(d, fmt, ...)                                \
	do {                                                      \
		if (debug)                                            \
			dev_info(razer_dev(d), "dbg: " fmt, ##__VA_ARGS__);\
		else                                                  \
			dev_dbg(razer_dev(d), fmt, ##__VA_ARGS__);         \
	} while (0)

// Hex dump only when debug=1 (so production logs stay clean)
static void razer_hex_dump(struct razer_pwm_dev *d, const char *prefix, const u8 *buf, size_t len)
{
	if (!debug)
		return;

	print_hex_dump(KERN_INFO, prefix, DUMP_PREFIX_OFFSET, 16, 1, buf, len, false);
}

struct razer_pwm_dev {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	struct mutex lock;

	// Cached state (hwmon expects "current value" semantics)
	u8 pwm_enable[CHANNEL_COUNT];   // 0/1
	u8 pwm_percent[CHANNEL_COUNT];  // 0..100

	// RPM cache (avoid hammering device)
	int rpm_cache[CHANNEL_COUNT];
	unsigned long rpm_cache_jiffies;

	struct delayed_work force_work;
};

// ===== Protocol helpers =====

// Checksum is XOR of bytes 3..88 (inclusive)
static u8 razer_checksum(const u8 *buf)
{
	int i;
	u8 c = 0;

	for (i = 3; i < 89; i++)
		c ^= buf[i];

	return c;
}

static void razer_build_packet(u8 *buf, u8 seq, u8 data_len, u8 cmd_class, u8 cmd)
{
	memset(buf, 0, PKT_LEN);
	buf[0] = 0x00;          // ReportId
	buf[1] = ST_DEFAULT;    // Status (host -> device)
	buf[2] = seq;
	buf[3] = 0x00;          // RemainingCount
	buf[4] = 0x00;
	buf[5] = 0x00;          // ProtocolType
	buf[6] = data_len;      // DataLength
	buf[7] = cmd_class;
	buf[8] = cmd;
	// buf[9..] filled by caller
	buf[89] = razer_checksum(buf);
	buf[90] = 0x00;         // Reserved
}

// Sequence increases by 0x08 each request (as per userland).
// 0x00 is avoided.
static u8 razer_next_seq(u8 *seq)
{
	do {
		*seq = (u8)(*seq + 0x08);
	} while (*seq == 0x00);
	return *seq;
}

static int razer_feature_set(struct hid_device *hdev, u8 report_id, u8 *buf, size_t len)
{
	// HID_FEATURE_REPORT + HID_REQ_SET_REPORT
	return hid_hw_raw_request(hdev, report_id, buf, len, HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
}

static int razer_feature_get(struct hid_device *hdev, u8 report_id, u8 *buf, size_t len)
{
	// HID_FEATURE_REPORT + HID_REQ_GET_REPORT
	return hid_hw_raw_request(hdev, report_id, buf, len, HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
}

// Write a packet and read the matching response.
// Returns 0 on success; negative errno otherwise.
//
// Behavior:
// - Any negative return from hid_hw_raw_request is an immediate error.
// - If we keep seeing BUSY until timeout, return -ETIMEDOUT.
// - If we receive a non-SUCCESS final status, return -EIO.
// - If sequence mismatches (stale response), we attempt a few quick reads to drain it.
static int razer_write_and_read(struct razer_pwm_dev *d, const u8 *tx, u8 *rx)
{
	int ret;
	unsigned long timeout;

	razer_dbg(d, "SET_REPORT seq=0x%02x class=0x%02x cmd=0x%02x len=%u\n",
		  tx[2], tx[7], tx[8], tx[6]);
	razer_hex_dump(d, "razer tx: ", tx, PKT_LEN);

	ret = razer_feature_set(d->hdev, tx[0], (u8 *)tx, PKT_LEN);
	if (ret < 0)
		return ret;

	msleep(DEVICE_READ_DELAY_MS);

	timeout = jiffies + msecs_to_jiffies(DEVICE_READ_TIMEOUT_MS);

	do {
		ret = razer_feature_get(d->hdev, tx[0], rx, PKT_LEN);
		if (ret < 0)
			return ret;
		razer_hex_dump(d, "razer rx: ", rx, (size_t)ret);

		// Some HID stacks could theoretically return a shorter read.
		if (ret < PKT_LEN) {
			msleep(DEVICE_READ_DELAY_MS);
			continue;
		}

		// Validate checksum (best-effort)
		if (rx[89] != razer_checksum(rx)) {
			razer_dbg(d, "checksum mismatch (rx=0x%02x calc=0x%02x)\n",
				  rx[89], razer_checksum(rx));
			// still continue; some stacks may mangle trailing bytes
		}

		// If BUSY, keep polling
		if (rx[1] == ST_BUSY) {
			msleep(DEVICE_READ_DELAY_MS);
			continue;
		}

		// If we got a stale packet (seq mismatch), drain a bit:
		if (rx[2] != tx[2]) {
			int i;
			for (i = 0; i < 5; i++) {
				msleep(DEVICE_READ_DELAY_MS);
				ret = razer_feature_get(d->hdev, tx[0], rx, PKT_LEN);
				if (ret < 0)
					return ret;

				if (ret >= 9 && rx[1] == ST_SUCCESS && rx[2] == tx[2])
					return 0;
			}

			hid_err(d->hdev, "sequence mismatch: expected 0x%02x got 0x%02x\n",
				tx[2], rx[2]);
			return -EIO;
		}

		break;

	} while (time_before(jiffies, timeout));

	if (rx[1] == ST_BUSY) {
		hid_err(d->hdev, "device BUSY timeout (seq=0x%02x)\n", tx[2]);
		return -ETIMEDOUT;
	}

	if (rx[1] != ST_SUCCESS) {
		hid_err(d->hdev, "device status error 0x%02x (seq=0x%02x)\n", rx[1], tx[2]);
		return -EIO;
	}

	return 0;
}

// ===== Device ops =====

static int razer_set_channel_mode_manual(struct razer_pwm_dev *d, int channel)
{
	u8 tx[PKT_LEN], rx[PKT_LEN];
	static u8 seq;

	razer_build_packet(tx, razer_next_seq(&seq), 3, CC_PWM, CMD_SET_CHANNEL_MODE);
	tx[9]  = 0x01;
	tx[10] = (u8)(0x05 + channel);
	tx[11] = MODE_MANUAL;
	tx[89] = razer_checksum(tx);

	return razer_write_and_read(d, tx, rx);
}

static int razer_set_channel_percent(struct razer_pwm_dev *d, int channel, u8 percent)
{
	u8 tx[PKT_LEN], rx[PKT_LEN];
	static u8 seq;

	if (percent > 100)
		percent = 100;

	razer_build_packet(tx, razer_next_seq(&seq), 3, CC_PWM, CMD_SET_CHANNEL_PERCENT);
	tx[9]  = 0x01;
	tx[10] = (u8)(0x05 + channel);
	tx[11] = percent;
	tx[89] = razer_checksum(tx);

	return razer_write_and_read(d, tx, rx);
}

static int razer_get_channel_speed(struct razer_pwm_dev *d, int channel, int *rpm_out)
{
	u8 tx[PKT_LEN], rx[PKT_LEN];
	static u8 seq;
	int ret;
	int rpm;

	razer_build_packet(tx, razer_next_seq(&seq), 6, CC_PWM, CMD_GET_CHANNEL_SPEED);
	tx[9]  = 0x01;
	tx[10] = (u8)(0x05 + channel);
	tx[89] = razer_checksum(tx);

	ret = razer_write_and_read(d, tx, rx);
	if (ret < 0)
		return ret;

	// Response stores RPM as big-endian int16 at Data[4..5] => rx[9+4], rx[9+5]
	rpm = ((int)rx[13] << 8) | (int)rx[14];
	*rpm_out = rpm;
	return 0;
}

static int razer_get_firmware(struct razer_pwm_dev *d, u8 *v1, u8 *v2, u8 *v3)
{
	u8 tx[PKT_LEN], rx[PKT_LEN];
	static u8 seq;
	int ret;

	razer_build_packet(tx, razer_next_seq(&seq), 0, CC_INFO, CMD_GET_FIRMWARE);
	tx[89] = razer_checksum(tx);

	ret = razer_write_and_read(d, tx, rx);
	if (ret < 0)
		return ret;

	// Firmware version bytes at Data[0..2] => rx[9..11]
	*v1 = rx[9];
	*v2 = rx[10];
	*v3 = rx[11];
	return 0;
}

// ===== hwmon callbacks =====

static int razer_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long *val)
{
	struct razer_pwm_dev *d = dev_get_drvdata(dev);
	int rpm, ret;

	mutex_lock(&d->lock);

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input || channel < 0 || channel >= CHANNEL_COUNT) {
			ret = -EINVAL;
			break;
		}

		// Simple cache: read at most every ~200ms
		if (time_before(jiffies, d->rpm_cache_jiffies + msecs_to_jiffies(200))) {
			*val = d->rpm_cache[channel];
			ret = 0;
			break;
		}

		ret = razer_get_channel_speed(d, channel, &rpm);
		if (ret == 0) {
			d->rpm_cache[channel] = rpm;
			d->rpm_cache_jiffies = jiffies;
			*val = rpm;
		}
		break;

	case hwmon_pwm:
		if (channel < 0 || channel >= CHANNEL_COUNT) {
			ret = -EINVAL;
			break;
		}

		if (attr == hwmon_pwm_input) {
			// hwmon expects 0..255; we store percent 0..100
			*val = (long)((d->pwm_percent[channel] * 255 + 50) / 100);
			ret = 0;
		} else if (attr == hwmon_pwm_enable) {
			*val = d->pwm_enable[channel];
			ret = 0;
		} else {
			ret = -EINVAL;
		}
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&d->lock);
	return ret;
}

static int razer_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long val)
{
	struct razer_pwm_dev *d = dev_get_drvdata(dev);
	int ret = 0;
	u8 percent;

	mutex_lock(&d->lock);

	switch (type) {
	case hwmon_pwm:
		if (channel < 0 || channel >= CHANNEL_COUNT) {
			ret = -EINVAL;
			break;
		}

		if (attr == hwmon_pwm_enable) {
			// Convention: 1=manual, 2=auto (not supported), 0=disable.
			// We implement:
			//  - 1: manual mode enabled
			//  - 0: set safe default 50% and mark disabled
			if (val == 1) {
				d->pwm_enable[channel] = 1;
				ret = razer_set_channel_mode_manual(d, channel);
				if (ret < 0)
					break;

				ret = razer_set_channel_percent(d, channel, d->pwm_percent[channel]);
			} else if (val == 0) {
				d->pwm_enable[channel] = 0;
				d->pwm_percent[channel] = DEFAULT_SPEED_PERCENT;
				ret = razer_set_channel_mode_manual(d, channel);
				if (ret < 0)
					break;
				ret = razer_set_channel_percent(d, channel, DEFAULT_SPEED_PERCENT);
			} else {
				ret = -EINVAL;
			}
		} else if (attr == hwmon_pwm_input) {
			// hwmon writes 0..255
			if (val < 0) val = 0;
			if (val > 255) val = 255;

			percent = (u8)((val * 100 + 127) / 255);
			d->pwm_percent[channel] = percent;

			// Apply only if manual enabled, otherwise just cache
			if (d->pwm_enable[channel] == 1) {
				ret = razer_set_channel_mode_manual(d, channel);
				if (ret < 0)
					break;
				ret = razer_set_channel_percent(d, channel, percent);
			}
		} else {
			ret = -EINVAL;
		}
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&d->lock);
	return ret;
}

static umode_t razer_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
				      u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input)
			return 0444;
		break;
	case hwmon_pwm:
		if (attr == hwmon_pwm_input)
			return 0644;
		if (attr == hwmon_pwm_enable)
			return 0644;
		break;
	default:
		break;
	}
	return 0;
}

static const struct hwmon_ops razer_hwmon_ops = {
	.is_visible = razer_hwmon_is_visible,
	.read = razer_hwmon_read,
	.write = razer_hwmon_write,
};

static const struct hwmon_channel_info *razer_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT,
			   HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT, HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
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

// Periodic "force write" worker.
// Re-applies cached PWM values (best effort).
// Why this exists: some userland implementations periodically rewrite PWM to keep the
// controller from reverting or drifting.
static void razer_force_workfn(struct work_struct *work)
{
	struct razer_pwm_dev *d = container_of(to_delayed_work(work), struct razer_pwm_dev, force_work);
	int i;

	if (force_write_ms <= 0)
		return;

	mutex_lock(&d->lock);

	for (i = 0; i < CHANNEL_COUNT; i++) {
		// Only re-apply in manual mode (1)
		if (d->pwm_enable[i] == 1) {
			(void)razer_set_channel_mode_manual(d, i);
			(void)razer_set_channel_percent(d, i, d->pwm_percent[i]);
		}
	}

	mutex_unlock(&d->lock);

	schedule_delayed_work(&d->force_work, msecs_to_jiffies(force_write_ms));
}

// ===== HID driver glue =====

static int razer_pwm_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct razer_pwm_dev *d;
	int ret, i;
	u8 fw1 = 0, fw2 = 0, fw3 = 0;

	d = devm_kzalloc(&hdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->hdev = hdev;
	mutex_init(&d->lock);
	INIT_DELAYED_WORK(&d->force_work, razer_force_workfn);

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

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hid_hw_open failed: %d\n", ret);
		goto stop;
	}

	// Initialize cache and put all channels in manual + safe default.
	mutex_lock(&d->lock);
	for (i = 0; i < CHANNEL_COUNT; i++) {
		d->pwm_enable[i]  = 0;
		d->pwm_percent[i] = DEFAULT_SPEED_PERCENT;
		d->rpm_cache[i]   = 0;
	}
	d->rpm_cache_jiffies = 0;

	for (i = 0; i < CHANNEL_COUNT; i++) {
		(void)razer_set_channel_mode_manual(d, i);
		(void)razer_set_channel_percent(d, i, DEFAULT_SPEED_PERCENT);
	}
	mutex_unlock(&d->lock);

	// Optional: read firmware for log/debug.
	if (razer_get_firmware(d, &fw1, &fw2, &fw3) == 0)
		hid_info(hdev, "Firmware: %u.%02u.%02u\n", fw1, fw2, fw3);

	// Register hwmon device
	d->hwmon_dev = devm_hwmon_device_register_with_info(&hdev->dev,
							    "razer_pwm_pc_fan",
							    d,
							    &razer_chip_info,
							    NULL);
	if (IS_ERR(d->hwmon_dev)) {
		ret = PTR_ERR(d->hwmon_dev);
		hid_err(hdev, "hwmon registration failed: %d\n", ret);
		goto close;
	}

	if (force_write_ms > 0)
		schedule_delayed_work(&d->force_work, msecs_to_jiffies(force_write_ms));

	hid_info(hdev, "Razer PWM PC Fan Controller hwmon loaded\n");
	return 0;

close:
	hid_hw_close(hdev);
stop:
	hid_hw_stop(hdev);
	return ret;
}

static void razer_pwm_remove(struct hid_device *hdev)
{
	struct razer_pwm_dev *d = hid_get_drvdata(hdev);

	if (d)
		cancel_delayed_work_sync(&d->force_work);

	hid_info(hdev, "Razer PWM PC Fan Controller hwmon removed\n");
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id razer_pwm_table[] = {
	{ HID_USB_DEVICE(0x1532, 0x0F3C) },
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SlimRG + contributors");
MODULE_DESCRIPTION("Razer PWM PC Fan Controller hwmon driver");

MODULE_ALIAS("razer_pwm_hwmon");
MODULE_ALIAS("razer_pwm_pc_fan");
