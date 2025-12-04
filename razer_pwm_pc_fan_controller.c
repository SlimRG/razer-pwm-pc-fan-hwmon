// SPDX-License-Identifier: GPL-2.0
/*
 * Razer PWM PC Fan Controller (USB HID) -> Linux hwmon driver
 *
 * Goal
 * ----
 * Provide a standard hwmon interface so users can control/read fans via:
 *   - lm-sensors ("sensors")
 *   - fancontrol/pwmconfig
 *   - any tool reading /sys/class/hwmon/hwmon*/{pwm*,fan*_input,...}
 *
 * Device
 * ------
 * Razer PWM PC Fan Controller: VID 0x1532, PID 0x0f3c
 * 8 channels (PWM outputs + tach inputs).
 *
 * Transport
 * ---------
 * The device uses HID Feature reports. We send a 91-byte "packet" via
 * HID_REQ_SET_REPORT and read the response via HID_REQ_GET_REPORT.
 *
 * Concurrency / Ownership
 * -----------------------
 * HID devices can be accessed by multiple parties. If another driver/userspace
 * talks to the device concurrently, you can get flaky results. This driver
 * serializes its own transactions with a mutex, but cannot prevent other
 * processes from sending feature reports. In practice you want this driver to
 * be the only owner (use driver_override/udev rule if needed).
 *
 * Stability feature: periodic "force write"
 * -----------------------------------------
 * Some implementations periodically re-apply current PWM settings to protect
 * against device internal resets or missed writes. We do the same via a
 * delayed_work that runs every 2500 ms and re-sends cached PWM values to all
 * channels that are enabled.
 *
 * Protocol summary
 * ----------------
 * Packet length: 91 bytes.
 * Fields:
 *  [0]  ReportId
 *  [1]  Status
 *  [2]  SequenceNumber (increments by 0x08, never 0x00)
 *  [3]  RemainingCount hi
 *  [4]  RemainingCount lo
 *  [5]  ProtocolType
 *  [6]  DataLength
 *  [7]  CommandClass
 *  [8]  Command
 *  [9..88] Data[80]
 *  [89] CRC (XOR bytes 3..88)
 *  [90] Reserved
 *
 * PWM addressing:
 *  Data[1] = 0x05 + channel (channel 0..7 => 0x05..0x0C)
 *
 * Commands:
 *  Info class 0x00 cmd 0x87: fw version in Data[0..2]
 *  PWM  class 0x0D cmd 0x02: set channel mode, Data[2]=0x04 => manual
 *  PWM  class 0x0D cmd 0x0D: set channel percent, Data[2]=0..100
 *  PWM  class 0x0D cmd 0x81: get channel speed, RPM in Data[4..5] (big-endian)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/workqueue.h>
#include <linux/err.h>

#define RZ_VENDOR_ID   0x1532
#define RZ_PRODUCT_ID  0x0f3c

/* Device has 8 PWM/tach channels. */
#define RZ_CHANNELS                 8

/* HID "packet" size used by the device protocol. */
#define RZ_PKT_LEN                  91
#define RZ_DATA_LEN                 80

/* Timing knobs for feature report exchange / busy-poll. */
#define RZ_DEVICE_READ_DELAY_MS     5
#define RZ_DEVICE_BUSY_TIMEOUT_MS   500

/* A conservative default PWM% we apply on load and when pwm_enable=0. */
#define RZ_DEFAULT_PERCENT          50

/*
 * Periodic "force write" interval:
 * Every 2500 ms we re-apply cached PWM% (manual mode) to all enabled channels.
 */
#define RZ_FORCE_WRITE_INTERVAL_MS  2500

/* Device response status values. */
enum rz_status : u8 {
	RZ_STATUS_DEFAULT = 0x00,
	RZ_STATUS_BUSY    = 0x01,
	RZ_STATUS_SUCCESS = 0x02,
	RZ_STATUS_ERROR   = 0x03,
	RZ_STATUS_TIMEOUT = 0x04,
	RZ_STATUS_INVALID = 0x05,
};

/* Command classes (protocol). */
#define RZ_CLASS_INFO               0x00
#define RZ_CLASS_PWM                0x0d

/* Info commands. */
#define RZ_CMD_INFO_FW              0x87

/* PWM commands. */
#define RZ_PWM_SET_MODE             0x02
#define RZ_PWM_SET_PERCENT          0x0d
#define RZ_PWM_GET_SPEED            0x81

/* Mode values (as observed in software implementations). */
#define RZ_MODE_MANUAL              0x04

/*
 * Device private state.
 *
 * Notes:
 * - io_lock serializes USB feature report transactions and also protects
 *   cached pwm_percent/pwm_enable from concurrent update vs force-work.
 * - force_work is the 2.5s periodic "re-apply current settings".
 */
struct rz_dev {
	struct hid_device *hdev;
	struct device *hwmon_dev;

	struct mutex io_lock;

	/* Protocol sequence counter (increments by 0x08, never 0). */
	u8 seq;

	/* Cached state (percent 0..100, enable 0/1). */
	u8 pwm_percent[RZ_CHANNELS];
	u8 pwm_enable[RZ_CHANNELS];

	/* Firmware string, optional. */
	char fw[16];
	bool have_fw;

	/* Periodic re-apply of PWM settings. */
	struct delayed_work force_work;

	/* Set to true during remove/unload to stop rescheduling. */
	bool stopping;
};

/* Compute XOR checksum over bytes 3..88 inclusive. */
static u8 rz_checksum_xor(const u8 *buf)
{
	u8 x = 0;
	int i;

	for (i = 3; i <= 88; i++)
		x ^= buf[i];

	return x;
}

/*
 * Advance and return the next sequence number.
 * The known implementations increment by 0x08 and never allow 0x00.
 */
static u8 rz_next_seq(struct rz_dev *d)
{
	d->seq += 0x08;
	if (d->seq == 0x00)
		d->seq += 0x08;
	return d->seq;
}

/*
 * Build a complete 91-byte packet.
 * 'data_len' controls how many bytes of Data[] are considered meaningful.
 * We still write all 80 bytes (zeros after data_len) for deterministic CRC.
 */
static void rz_build_packet(struct rz_dev *d, u8 *pkt,
			    u8 cmd_class, u8 cmd, u8 data_len,
			    const u8 *data /* may be NULL */)
{
	int i;

	memset(pkt, 0, RZ_PKT_LEN);

	/* Header */
	pkt[0] = 0x00;                 /* ReportId (commonly 0) */
	pkt[1] = RZ_STATUS_DEFAULT;    /* Status in request */
	pkt[2] = rz_next_seq(d);       /* SequenceNumber */
	pkt[3] = 0x00;                 /* RemainingCount (hi) */
	pkt[4] = 0x00;                 /* RemainingCount (lo) */
	pkt[5] = 0x00;                 /* ProtocolType */
	pkt[6] = data_len;             /* DataLength */
	pkt[7] = cmd_class;            /* CommandClass */
	pkt[8] = cmd;                  /* Command */

	/* Data payload (80 bytes) */
	for (i = 0; i < RZ_DATA_LEN; i++)
		pkt[9 + i] = (data && i < data_len) ? data[i] : 0x00;

	/* CRC and reserved */
	pkt[89] = rz_checksum_xor(pkt);
	pkt[90] = 0x00;
}

/*
 * Wrapper for HID feature report GET.
 * We use hid_hw_raw_request() with HID_FEATURE_REPORT / HID_REQ_GET_REPORT.
 *
 * Note: Many HID devices expect the report ID to be present in buf[0].
 */
static int rz_get_feature(struct rz_dev *d, u8 report_id, u8 *buf, size_t len)
{
	int ret;

	buf[0] = report_id;
	ret = hid_hw_raw_request(d->hdev, report_id, buf, len,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);

	/* ret is number of bytes transferred or negative errno */
	return (ret < 0) ? ret : 0;
}

/* Wrapper for HID feature report SET. */
static int rz_set_feature(struct rz_dev *d, u8 report_id, u8 *buf, size_t len)
{
	int ret;

	ret = hid_hw_raw_request(d->hdev, report_id, buf, len,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	return (ret < 0) ? ret : 0;
}

/*
 * Perform a full transaction:
 *   - SET_FEATURE(tx)
 *   - short sleep
 *   - GET_FEATURE(rx)
 *   - if rx status == BUSY, keep polling GET_FEATURE until timeout
 *
 * Important:
 * - Must be called from sleeping context (we msleep).
 * - Serialized by io_lock so multiple sysfs accesses don't interleave.
 */
static int rz_xfer_locked(struct rz_dev *d, u8 *tx, u8 *rx)
{
	int ret;
	unsigned long deadline;

	/* SET */
	ret = rz_set_feature(d, tx[0], tx, RZ_PKT_LEN);
	if (ret)
		return ret;

	msleep(RZ_DEVICE_READ_DELAY_MS);

	/* First GET */
	deadline = jiffies + msecs_to_jiffies(RZ_DEVICE_BUSY_TIMEOUT_MS);

	ret = rz_get_feature(d, tx[0], rx, RZ_PKT_LEN);
	if (ret)
		return ret;

	/* BUSY polling loop */
	while (rx[1] == RZ_STATUS_BUSY && time_before(jiffies, deadline)) {
		msleep(RZ_DEVICE_READ_DELAY_MS);
		ret = rz_get_feature(d, tx[0], rx, RZ_PKT_LEN);
		if (ret)
			return ret;
	}

	/*
	 * Some firmwares might report DEFAULT or SUCCESS on read-type commands.
	 * For write commands we prefer SUCCESS, but we keep it tolerant here.
	 */
	return 0;
}

/* Same as rz_xfer_locked(), but manages io_lock itself. */
static int rz_xfer(struct rz_dev *d, u8 *tx, u8 *rx)
{
	int ret;

	mutex_lock(&d->io_lock);
	ret = rz_xfer_locked(d, tx, rx);
	mutex_unlock(&d->io_lock);

	return ret;
}

/* --- Protocol helpers: set manual mode, set percent, read RPM, read FW --- */

static int rz_cmd_set_manual_locked(struct rz_dev *d, int ch)
{
	u8 data[3];
	u8 tx[RZ_PKT_LEN], rx[RZ_PKT_LEN];

	data[0] = 0x01;
	data[1] = (u8)(0x05 + ch);
	data[2] = RZ_MODE_MANUAL;

	rz_build_packet(d, tx, RZ_CLASS_PWM, RZ_PWM_SET_MODE, 3, data);
	return rz_xfer_locked(d, tx, rx);
}

static int rz_cmd_set_percent_locked(struct rz_dev *d, int ch, u8 percent)
{
	u8 data[3];
	u8 tx[RZ_PKT_LEN], rx[RZ_PKT_LEN];

	data[0] = 0x01;
	data[1] = (u8)(0x05 + ch);
	data[2] = percent;

	rz_build_packet(d, tx, RZ_CLASS_PWM, RZ_PWM_SET_PERCENT, 3, data);
	return rz_xfer_locked(d, tx, rx);
}

/*
 * Read RPM for a given channel. Response encodes RPM in Data[4..5] big-endian.
 */
static int rz_cmd_get_rpm(struct rz_dev *d, int ch, u16 *rpm)
{
	u8 data[6];
	u8 tx[RZ_PKT_LEN], rx[RZ_PKT_LEN];
	int ret;

	memset(data, 0, sizeof(data));
	data[0] = 0x01;
	data[1] = (u8)(0x05 + ch);

	rz_build_packet(d, tx, RZ_CLASS_PWM, RZ_PWM_GET_SPEED, 6, data);

	ret = rz_xfer(d, tx, rx);
	if (ret)
		return ret;

	*rpm = ((u16)rx[9 + 4] << 8) | (u16)rx[9 + 5];
	return 0;
}

/* Try to read firmware version (optional; failure is non-fatal). */
static int rz_try_read_fw(struct rz_dev *d)
{
	u8 tx[RZ_PKT_LEN], rx[RZ_PKT_LEN];
	int ret, i;
	u8 major, minor, patch;

	rz_build_packet(d, tx, RZ_CLASS_INFO, RZ_CMD_INFO_FW, 0, NULL);

	for (i = 0; i < 10; i++) {
		ret = rz_xfer(d, tx, rx);
		if (!ret) {
			major = rx[9 + 0];
			minor = rx[9 + 1];
			patch = rx[9 + 2];
			snprintf(d->fw, sizeof(d->fw), "%u.%02u.%02u", major, minor, patch);
			d->have_fw = true;
			return 0;
		}
		msleep(20);
	}

	return ret ? ret : -EIO;
}

/*
 * Apply (manual + percent) to a channel.
 * Called by sysfs writes and by periodic force-work.
 * Must be called with io_lock held if you want atomicity vs other operations.
 */
static int rz_apply_percent_locked(struct rz_dev *d, int ch, u8 percent)
{
	int ret;

	ret = rz_cmd_set_manual_locked(d, ch);
	if (ret)
		return ret;

	ret = rz_cmd_set_percent_locked(d, ch, percent);
	if (ret)
		return ret;

	d->pwm_percent[ch] = percent;
	d->pwm_enable[ch] = 1;
	return 0;
}

/* -------------- periodic force write (every 2500 ms) -------------- */

/*
 * Periodic worker:
 * Re-applies cached PWM settings to protect against device state drift.
 *
 * Design choices:
 * - Run in process context (workqueue) so sleeping/msleep is allowed.
 * - Hold io_lock for the durations of all channel writes to avoid interleaving
 *   from concurrent sysfs access while we "refresh" the device.
 * - Re-schedule itself unless we're stopping.
 */
static void rz_force_write_workfn(struct work_struct *work)
{
	struct rz_dev *d = container_of(to_delayed_work(work), struct rz_dev, force_work);
	int ch;

	mutex_lock(&d->io_lock);

	if (!d->stopping) {
		for (ch = 0; ch < RZ_CHANNELS; ch++) {
			if (d->pwm_enable[ch]) {
				/* Best-effort: ignore errors to keep refreshing other channels. */
				(void)rz_apply_percent_locked(d, ch, d->pwm_percent[ch]);
			}
		}
	}

	mutex_unlock(&d->io_lock);

	/* Reschedule if still active */
	if (!d->stopping)
		schedule_delayed_work(&d->force_work, msecs_to_jiffies(RZ_FORCE_WRITE_INTERVAL_MS));
}

/* ---------------- hwmon callbacks (read/write sysfs) ---------------- */

static int rz_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct rz_dev *d = dev_get_drvdata(dev);
	u16 rpm;
	int ret;
	u8 percent, pwm8;

	if (channel < 0 || channel >= RZ_CHANNELS)
		return -EINVAL;

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input)
			return -EOPNOTSUPP;

		ret = rz_cmd_get_rpm(d, channel, &rpm);
		if (ret)
			return ret;

		*val = (long)rpm;
		return 0;

	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			/*
			 * hwmon PWM convention is often 0..255. Internally we store 0..100%.
			 * Convert with rounding.
			 */
			mutex_lock(&d->io_lock);
			percent = d->pwm_percent[channel];
			mutex_unlock(&d->io_lock);

			pwm8 = (u8)DIV_ROUND_CLOSEST((int)percent * 255, 100);
			*val = (long)pwm8;
			return 0;

		case hwmon_pwm_enable:
			/*
			 * 1 = "enabled"/manual from our perspective,
			 * 0 = "default" (we implement as setting default percent).
			 */
			mutex_lock(&d->io_lock);
			*val = (long)d->pwm_enable[channel];
			mutex_unlock(&d->io_lock);
			return 0;

		default:
			return -EOPNOTSUPP;
		}

	default:
		return -EOPNOTSUPP;
	}
}

static int rz_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct rz_dev *d = dev_get_drvdata(dev);
	u8 percent;
	int ret;

	if (channel < 0 || channel >= RZ_CHANNELS)
		return -EINVAL;

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			/*
			 * Userspace writes pwmN as 0..255.
			 * Convert to 0..100% with rounding, then apply to device.
			 */
			if (val < 0)
				val = 0;
			if (val > 255)
				val = 255;

			percent = (u8)DIV_ROUND_CLOSEST((int)val * 100, 255);
			if (percent > 100)
				percent = 100;

			mutex_lock(&d->io_lock);
			ret = rz_apply_percent_locked(d, channel, percent);
			mutex_unlock(&d->io_lock);
			return ret;

		case hwmon_pwm_enable:
			/*
			 * We do not implement a hardware "auto" mode because public protocol
			 * examples only show manual mode (0x04). Therefore:
			 *   - writing 1: we mark enabled; device stays at current cached percent
			 *   - writing 0: we set default percent (RZ_DEFAULT_PERCENT)
			 */
			if (val == 1) {
				mutex_lock(&d->io_lock);
				d->pwm_enable[channel] = 1;
				mutex_unlock(&d->io_lock);
				return 0;
			}

			if (val == 0) {
				mutex_lock(&d->io_lock);
				ret = rz_apply_percent_locked(d, channel, RZ_DEFAULT_PERCENT);
				mutex_unlock(&d->io_lock);
				return ret;
			}

			return -EINVAL;

		default:
			return -EOPNOTSUPP;
		}

	default:
		return -EOPNOTSUPP;
	}
}

static umode_t rz_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	if (channel < 0 || channel >= RZ_CHANNELS)
		return 0;

	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input)
			return 0444;
		return 0;

	case hwmon_pwm:
		if (attr == hwmon_pwm_input || attr == hwmon_pwm_enable)
			return 0644;
		return 0;

	default:
		return 0;
	}
}

static const struct hwmon_ops rz_hwmon_ops = {
	.is_visible = rz_hwmon_is_visible,
	.read = rz_hwmon_read,
	.write = rz_hwmon_write,
};

static const struct hwmon_channel_info *rz_hwmon_info[] = {
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

static const struct hwmon_chip_info rz_chip_info = {
	.ops = &rz_hwmon_ops,
	.info = rz_hwmon_info,
};

/* ---------------- HID probe/remove ---------------- */

static int rz_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct rz_dev *d;
	struct device *hwmon;
	int ret, ch;

	d = devm_kzalloc(&hdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->hdev = hdev;
	mutex_init(&d->io_lock);
	d->seq = 0x00;
	d->stopping = false;

	/* Initialize cached state (defaults). */
	for (ch = 0; ch < RZ_CHANNELS; ch++) {
		d->pwm_percent[ch] = RZ_DEFAULT_PERCENT;
		d->pwm_enable[ch] = 1;
	}

	hid_set_drvdata(hdev, d);

	/* Parse HID descriptor and start the HID device. */
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

	/*
	 * Open the device for raw requests. Some stacks/devices behave better
	 * with explicit open/close around raw requests.
	 */
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hid_hw_open failed: %d\n", ret);
		hid_hw_stop(hdev);
		return ret;
	}

	/* Optional firmware read. */
	if (!rz_try_read_fw(d))
		hid_info(hdev, "Firmware: %s\n", d->fw);

	/* Apply defaults on probe (best-effort). */
	mutex_lock(&d->io_lock);
	for (ch = 0; ch < RZ_CHANNELS; ch++)
		(void)rz_apply_percent_locked(d, ch, RZ_DEFAULT_PERCENT);
	mutex_unlock(&d->io_lock);

	/* Register hwmon device (creates sysfs nodes). */
	hwmon = devm_hwmon_device_register_with_info(&hdev->dev,
						     "razer_pwm_pc_fan",
						     d,
						     &rz_chip_info,
						     NULL);
	if (IS_ERR(hwmon)) {
		ret = PTR_ERR(hwmon);
		hid_err(hdev, "hwmon register failed: %d\n", ret);
		hid_hw_close(hdev);
		hid_hw_stop(hdev);
		return ret;
	}

	d->hwmon_dev = hwmon;
	dev_set_drvdata(hwmon, d);

	/*
	 * Start periodic force write loop.
	 * We use the global workqueue via schedule_delayed_work().
	 */
	INIT_DELAYED_WORK(&d->force_work, rz_force_write_workfn);
	schedule_delayed_work(&d->force_work, msecs_to_jiffies(RZ_FORCE_WRITE_INTERVAL_MS));

	hid_info(hdev, "Razer PWM PC Fan Controller hwmon ready (force-write %d ms)\n",
		 RZ_FORCE_WRITE_INTERVAL_MS);
	return 0;
}

static void rz_remove(struct hid_device *hdev)
{
	struct rz_dev *d = hid_get_drvdata(hdev);
	int ch;

	if (d) {
		/* Stop periodic worker safely. */
		mutex_lock(&d->io_lock);
		d->stopping = true;
		mutex_unlock(&d->io_lock);
		cancel_delayed_work_sync(&d->force_work);

		/* Best-effort: set safe defaults on unload. */
		mutex_lock(&d->io_lock);
		for (ch = 0; ch < RZ_CHANNELS; ch++)
			(void)rz_apply_percent_locked(d, ch, RZ_DEFAULT_PERCENT);
		mutex_unlock(&d->io_lock);
	}

	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id rz_hid_table[] = {
	{ HID_USB_DEVICE(RZ_VENDOR_ID, RZ_PRODUCT_ID) },
	{ }
};
MODULE_DEVICE_TABLE(hid, rz_hid_table);

static struct hid_driver rz_driver = {
	.name = "razer_pwm_pc_fan",
	.id_table = rz_hid_table,
	.probe = rz_probe,
	.remove = rz_remove,
};

module_hid_driver(rz_driver);

MODULE_AUTHOR("SlimRG-ready out-of-tree example");
MODULE_DESCRIPTION("Razer PWM PC Fan Controller (HID) -> hwmon fan/pwm driver with periodic force-write");
MODULE_LICENSE("GPL");
