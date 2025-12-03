/*
 * MGC3130 Gesture Sensor Driver
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "mgc_handler.h"

LOG_MODULE_REGISTER(mgc_handler, LOG_LEVEL_INF);

/* MGC3130 I2C Configuration */
#define MGC3130_I2C_ADDR  0x42

/* GPIO Pin Definitions */
#define MGC3130_MCLR_PIN  28  /* Reset pin (active low) */
#define MGC3130_TS_PIN    29  /* Transfer Status pin */

/* Timing Constants */
#define MGC3130_RESET_DELAY_MS        	10
#define MGC3130_BOOT_DELAY_MS         	100
#define MGC3130_POST_TRANSFER_DELAY_US 	200
#define MGC3130_TS_TIMEOUT_MS         	500

/* Message Buffer Size */
#define MGC3130_MSG_BUF_SIZE  256

/* Driver Instance */
typedef struct {
	driver_instance_t base;
	const struct device *i2c_dev;
	const struct device *gpio_dev;
	mgc3130_fw_version_t fw_version;
	bool fw_version_valid;
} mgc_handler_instance_t;

static mgc_handler_instance_t mgc_inst = {
	.base = {
		.state = DRIVER_STATE_CLOSED,
		.flags = 0,
	},
	.fw_version_valid = false,
};

/* Request Message to request Fw_Version_Info */
static const uint8_t request_fw_version[] = {
	0x0C, 0x00,  /* Size: 12 bytes (little-endian) */
	0x00,        /* Flags */
	0x06,        /* ID: Request_Message */
	0x83,        /* Payload: request FW_Version_Info (0x83) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Helper Functions */

/*
 * Wait for TS line to go low (data ready)
 * Returns: 0 on success, -ETIMEDOUT on timeout
 */
static int wait_for_ts_ready(uint32_t timeout_ms)
{
	int64_t end_time = k_uptime_get() + timeout_ms;
	int value;

	while (k_uptime_get() < end_time) {
		value = gpio_pin_get(mgc_inst.gpio_dev, MGC3130_TS_PIN);
		if (value < 0) {
			LOG_ERR("Failed to read TS pin: %d", value);
			return value;
		}
		if (value == 0) {
			/* TS is low - data ready */
			return 0;
		}
		k_sleep(K_MSEC(1));
	}

	LOG_WRN("TS line timeout");
	return -ETIMEDOUT;
}

/*
 * Read message from MGC3130 via I2C with TS protocol
 * Returns: number of bytes read on success, negative error code on failure
 */
static int mgc3130_read_message(uint8_t *buf, size_t max_len)
{
	int ret;
	uint16_t msg_size;

	/* Wait for TS low (data ready) */
	ret = wait_for_ts_ready(MGC3130_TS_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	/* Read message via I2C (this asserts TS high via START condition) */
	ret = i2c_read(mgc_inst.i2c_dev, buf, max_len, MGC3130_I2C_ADDR);
	if (ret < 0) {
		LOG_ERR("I2C read failed: %d", ret);
		return ret;
	}

	/* I2C STOP condition releases TS line */

	/* Parse message size from header (little-endian) */
	msg_size = buf[0] | (buf[1] << 8);

	/* Mandatory 200μs wait after transfer */
	k_usleep(MGC3130_POST_TRANSFER_DELAY_US);

	LOG_DBG("Read message: size=%u, ID=0x%02X", msg_size, buf[3]);

	return msg_size;
}

/*
 * Write message to MGC3130 via I2C
 * Returns: 0 on success, negative error code on failure
 */
static int mgc3130_write_message(const uint8_t *buf, size_t len)
{
	int ret;

	/* Write message via I2C */
	ret = i2c_write(mgc_inst.i2c_dev, buf, len, MGC3130_I2C_ADDR);
	if (ret < 0) {
		LOG_ERR("I2C write failed: %d", ret);
		return ret;
	}

	/* Mandatory 200μs wait after transfer */
	k_usleep(MGC3130_POST_TRANSFER_DELAY_US);

	LOG_DBG("Write message: len=%u", len);

	return 0;
}

/*
 * Reset MGC3130 using MCLR pin
 */
static int mgc3130_reset(void)
{
	int ret;

	LOG_INF("Resetting MGC3130...");

	/* Assert MCLR low (reset active) */
	ret = gpio_pin_set(mgc_inst.gpio_dev, MGC3130_MCLR_PIN, 0);
	if (ret < 0) {
		LOG_ERR("Failed to assert MCLR: %d", ret);
		return ret;
	}

	k_msleep(MGC3130_RESET_DELAY_MS);

	/* Release MCLR high (reset inactive) */
	ret = gpio_pin_set(mgc_inst.gpio_dev, MGC3130_MCLR_PIN, 1);
	if (ret < 0) {
		LOG_ERR("Failed to release MCLR: %d", ret);
		return ret;
	}

	/* Wait for MGC3130 to boot */
	k_msleep(MGC3130_BOOT_DELAY_MS);

	LOG_INF("MGC3130 reset complete");

	return 0;
}

/*
 * Parse firmware version string to extract individual fields
 * Format: "1.3.14;p:HillstarV01;x:HTIS_GS;DSP:ID9000r2963;i:B;f:22500;nMsg;s:Rel_1_3;t:2013/11/08 13:03:08;..."
 */
static void parse_fw_version_string(mgc3130_fw_version_t *ver)
{
	char *str = ver->version_string;
	char *field_start;
	char *field_end;
	size_t field_len;

	/* Initialize all fields to empty strings */
	ver->library_version[0] = '\0';
	ver->platform[0] = '\0';
	ver->colibri_version[0] = '\0';
	ver->build_time[0] = '\0';

	/* Extract GestIC Library Version (everything before first semicolon) */
	field_end = strchr(str, ';');
	if (field_end != NULL) {
		field_len = field_end - str;
		if (field_len < sizeof(ver->library_version)) {
			memcpy(ver->library_version, str, field_len);
			ver->library_version[field_len] = '\0';
		}
	} else {
		/* No semicolon found, use entire string */
		strncpy(ver->library_version, str, sizeof(ver->library_version) - 1);
		return;
	}

	/* Extract Platform (p:) */
	field_start = strstr(str, "p:");
	if (field_start != NULL) {
		field_start += 2;  /* Skip "p:" */
		field_end = strchr(field_start, ';');
		if (field_end != NULL) {
			field_len = field_end - field_start;
		} else {
			field_len = strlen(field_start);
		}
		if (field_len < sizeof(ver->platform)) {
			memcpy(ver->platform, field_start, field_len);
			ver->platform[field_len] = '\0';
		}
	}

	/* Extract Colibri Suite Version (DSP:) */
	field_start = strstr(str, "DSP:");
	if (field_start != NULL) {
		field_start += 4;  /* Skip "DSP:" */
		field_end = strchr(field_start, ';');
		if (field_end != NULL) {
			field_len = field_end - field_start;
		} else {
			field_len = strlen(field_start);
		}
		if (field_len < sizeof(ver->colibri_version)) {
			memcpy(ver->colibri_version, field_start, field_len);
			ver->colibri_version[field_len] = '\0';
		}
	}

	/* Extract Build Time (t:) */
	field_start = strstr(str, "t:");
	if (field_start != NULL) {
		field_start += 2;  /* Skip "t:" */
		field_end = strchr(field_start, ';');
		if (field_end != NULL) {
			field_len = field_end - field_start;
		} else {
			field_len = strlen(field_start);
		}
		if (field_len < sizeof(ver->build_time)) {
			memcpy(ver->build_time, field_start, field_len);
			ver->build_time[field_len] = '\0';
		}
	}
}

/*
 * Get firmware version from MGC3130
 * The device sends FW_Version_Info automatically at startup,
 * or we can request it using Request_Message
 */
static int mgc3130_get_fw_version(mgc3130_fw_version_t *ver)
{
	uint8_t buf[MGC3130_MSG_BUF_SIZE];
	int ret;
	uint8_t msg_id;
	uint16_t msg_size;
	size_t payload_offset;
	size_t payload_len;

	/* Wait for automatic FW_Version_Info message after reset */
	ret = wait_for_ts_ready(MGC3130_TS_TIMEOUT_MS);
	if (ret < 0) {
		LOG_INF("No automatic FW version, sending request...");

		/* Send Request_Message to request FW version */
		ret = mgc3130_write_message(request_fw_version, sizeof(request_fw_version));
		if (ret < 0) {
			LOG_ERR("Failed to send request message");
			return ret;
		}

		/* Wait for response */
		ret = wait_for_ts_ready(MGC3130_TS_TIMEOUT_MS);
		if (ret < 0) {
			LOG_ERR("No response to request message");
			return ret;
		}
	}

	/* Read FW_Version_Info message */
	ret = mgc3130_read_message(buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read FW version message");
		return ret;
	}

	msg_size = buf[0] | (buf[1] << 8);
	msg_id = buf[3];

	/* Validate message ID */
	if (msg_id != MGC3130_MSG_FW_VERSION_INFO) {
		LOG_ERR("Unexpected message ID: 0x%02X (expected 0x83)", msg_id);
		return -EINVAL;
	}

	LOG_INF("Received FW_Version_Info message (size=%u)", msg_size);

	/* Extract payload - starts after header
	 * Header format: Size(2) + Flags(1) + ID(1) = 4 bytes minimum
	 * Actual message may have additional header bytes
	 */
	payload_offset = 4;  /* Skip standard 4-byte header */

	/* Find start of version string in payload (starts with a digit 0-9) */
	while (payload_offset < msg_size && payload_offset < sizeof(buf)) {
		/* Version string starts with a digit (e.g., "1.0.0" or "1.3.14") */
		if (buf[payload_offset] >= '0' && buf[payload_offset] <= '9') {
			break;
		}
		payload_offset++;
	}

	if (payload_offset >= msg_size) {
		LOG_ERR("No version string found in message");
		return -EINVAL;
	}

	payload_len = msg_size - payload_offset;
	if (payload_len > MGC3130_FW_VERSION_MAX_LEN - 1) {
		payload_len = MGC3130_FW_VERSION_MAX_LEN - 1;
	}

	/* Copy ASCII string to version structure */
	memcpy(ver->version_string, &buf[payload_offset], payload_len);
	ver->version_string[payload_len] = '\0';  /* Null-terminate */
	ver->length = payload_len;

	/* Parse the version string into structured fields */
	parse_fw_version_string(ver);

	LOG_INF("GestIC Library Version: %s", ver->library_version);
	LOG_INF("Platform: %s", ver->platform);
	LOG_INF("Colibri Suite Version: %s", ver->colibri_version);
	if (ver->build_time[0] != '\0') {
		LOG_INF("Build Time: %s", ver->build_time);
	}

	/* Read System_Status message if present */
	ret = wait_for_ts_ready(100);  /* Short timeout */
	if (ret == 0) {
		ret = mgc3130_read_message(buf, sizeof(buf));
		if (ret > 0) {
			msg_id = buf[3];
			if (msg_id == MGC3130_MSG_SYSTEM_STATUS) {
				LOG_INF("Received System_Status message");
			}
		}
	}

	return 0;
}

/* Driver API Implementation */

driver_fd_t mgc_open(void)
{
	DRIVER_LOCK(&mgc_inst.base);

	if (mgc_inst.base.state == DRIVER_STATE_OPEN) {
		LOG_WRN("Driver already open");
		DRIVER_UNLOCK(&mgc_inst.base);
		return -EBUSY;
	}

	if (mgc_inst.base.state == DRIVER_STATE_ERROR) {
		LOG_ERR("Driver in error state");
		DRIVER_UNLOCK(&mgc_inst.base);
		return -EIO;
	}

	mgc_inst.base.state = DRIVER_STATE_OPEN;

	DRIVER_UNLOCK(&mgc_inst.base);

	LOG_INF("MGC handler opened");
	return 0;
}

int mgc_close(driver_fd_t fd)
{
	if (mgc_inst.base.state != DRIVER_STATE_OPEN) {
		LOG_ERR("Driver not open");
		return -EINVAL;
	}

	DRIVER_LOCK(&mgc_inst.base);

	mgc_inst.base.state = DRIVER_STATE_CLOSED;

	DRIVER_UNLOCK(&mgc_inst.base);

	LOG_INF("MGC handler closed");
	return 0;
}

ssize_t mgc_read(driver_fd_t fd, void *buf, size_t count)
{
	int ret;

	if (mgc_inst.base.state != DRIVER_STATE_OPEN) {
		LOG_ERR("Driver not open");
		return -EINVAL;
	}

	if (buf == NULL || count == 0) {
		return -EINVAL;
	}

	DRIVER_LOCK(&mgc_inst.base);

	/* Read raw message from MGC3130 */
	ret = mgc3130_read_message((uint8_t *)buf, count);

	DRIVER_UNLOCK(&mgc_inst.base);

	return ret;
}

int mgc_ioctl(driver_fd_t fd, unsigned int cmd, void *arg)
{
	int ret = 0;

	if (mgc_inst.base.state != DRIVER_STATE_OPEN) {
		LOG_ERR("Driver not open");
		return -EINVAL;
	}

	DRIVER_LOCK(&mgc_inst.base);

	switch (cmd) {
	case MGC_IOCTL_GET_FW_VERSION:
		if (arg == NULL) {
			ret = -EINVAL;
			break;
		}

		if (mgc_inst.fw_version_valid) {
			/* Return cached version */
			memcpy(arg, &mgc_inst.fw_version, sizeof(mgc3130_fw_version_t));
		} else {
			/* Read version from device */
			ret = mgc3130_get_fw_version((mgc3130_fw_version_t *)arg);
			if (ret == 0) {
				/* Cache the version */
				memcpy(&mgc_inst.fw_version, arg, sizeof(mgc3130_fw_version_t));
				mgc_inst.fw_version_valid = true;
			}
		}
		break;

	default:
		LOG_WRN("Unknown ioctl command: 0x%02X", cmd);
		ret = -EINVAL;
		break;
	}

	DRIVER_UNLOCK(&mgc_inst.base);

	return ret;
}

/* Driver Initialization */

static int mgc_init(void)
{
	int ret;

	LOG_INF("Initializing MGC3130 handler...");

	/* Initialize mutex */
	k_mutex_init(&mgc_inst.base.lock);

	/* Get I2C1 device */
	mgc_inst.i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(mgc_inst.i2c_dev)) {
		LOG_ERR("I2C1 device not ready");
		return -ENODEV;
	}
	LOG_INF("I2C1 device ready");

	/* Get GPIO device */
	mgc_inst.gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(mgc_inst.gpio_dev)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}
	LOG_INF("GPIO device ready");

	/* Configure MCLR pin as output (initially high - reset inactive) */
	ret = gpio_pin_configure(mgc_inst.gpio_dev, MGC3130_MCLR_PIN,
	                         GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_HIGH);
	if (ret < 0) {
		LOG_ERR("Failed to configure MCLR pin: %d", ret);
		return ret;
	}

	/* Configure TS pin as input */
	ret = gpio_pin_configure(mgc_inst.gpio_dev, MGC3130_TS_PIN,
	                         GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure TS pin: %d", ret);
		return ret;
	}

	LOG_INF("GPIO pins configured (MCLR=%d, TS=%d)", MGC3130_MCLR_PIN, MGC3130_TS_PIN);

	/* Perform reset sequence */
	ret = mgc3130_reset();
	if (ret < 0) {
		LOG_ERR("Reset failed: %d", ret);
		return ret;
	}

	/* Try to read firmware version */
	ret = mgc3130_get_fw_version(&mgc_inst.fw_version);
	if (ret == 0) {
		mgc_inst.fw_version_valid = true;
		LOG_INF("Initial FW version read successful");
	} else {
		LOG_WRN("Could not read initial FW version (will retry on first ioctl)");
		/* Don't fail initialization - we can retry later */
	}

	/* State remains DRIVER_STATE_CLOSED - application must call open() */

	LOG_INF("MGC3130 handler initialized successfully");

	return 0;
}

SYS_INIT(mgc_init, APPLICATION, 92);
