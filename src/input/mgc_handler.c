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
	mgc3130_config_t config;           /* Configuration state */
	mgc3130_touch_state_t touch_state; /* Touch tracking state */
	bool configured;                    /* Runtime parameters configured */
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
 * Set a runtime parameter on the MGC3130
 *
 * @param param_id: Runtime parameter ID (e.g., 0x0097)
 * @param arg0: Argument 0
 * @param arg1: Argument 1
 * @return: 0 on success, negative error code on failure
 */
static int mgc3130_set_runtime_parameter(uint16_t param_id, uint32_t arg0, uint32_t arg1)
{
	uint8_t buf[MGC3130_MSG_BUF_SIZE];
	mgc3130_set_runtime_param_msg_t msg;
	mgc3130_system_status_msg_t *status;
	int ret;
	uint16_t msg_size;
	uint8_t msg_id;

	/* Build SET_RUNTIME_PARAMETER message */
	msg.header.size = sizeof(mgc3130_set_runtime_param_msg_t);
	msg.header.flags = 0;
	msg.header.seq = 0;
	msg.header.id = MGC3130_MSG_SET_RUNTIME_PARAMETER;
	msg.param_id = param_id;
	msg.reserved = 0;
	msg.arg0 = arg0;
	msg.arg1 = arg1;

	LOG_DBG("Setting runtime param 0x%04X: arg0=0x%08X, arg1=0x%08X",
	        param_id, arg0, arg1);

	/* Send message */
	ret = mgc3130_write_message((uint8_t *)&msg, sizeof(msg));
	if (ret < 0) {
		LOG_ERR("Failed to write SET_RUNTIME_PARAMETER");
		return ret;
	}

	/* Wait for System_Status response */
	ret = mgc3130_read_message(buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read System_Status response");
		return ret;
	}

	msg_size = buf[0] | (buf[1] << 8);
	msg_id = buf[3];

	/* Validate response is System_Status */
	if (msg_id != MGC3130_MSG_SYSTEM_STATUS) {
		LOG_ERR("Unexpected response ID: 0x%02X (expected 0x15)", msg_id);
		return -EINVAL;
	}

	/* Parse System_Status */
	status = (mgc3130_system_status_msg_t *)buf;

	if (status->error_code != 0) {
		LOG_ERR("Parameter set failed with error code: 0x%02X", status->error_code);
		return -EIO;
	}

	LOG_DBG("Runtime parameter 0x%04X set successfully", param_id);
	return 0;
}

/*
 * Configure MGC3130 for touch detection
 * Enables touch detection and TouchInfo output
 *
 * @return: 0 on success, negative error code on failure
 */
static int mgc3130_configure_touch(void)
{
	int ret;

	LOG_INF("Configuring MGC3130 for touch detection...");

	/* Step 1: Enable touch detection (dspTouchConfig = 0x0097) */
	ret = mgc3130_set_runtime_parameter(
		MGC3130_PARAM_DSP_TOUCH_CONFIG,
		0x08,        /* arg0: 0x08 = enable touch */
		0x08         /* arg1: 0x08 = required value */
	);
	if (ret < 0) {
		LOG_ERR("Failed to enable touch detection: %d", ret);
		return ret;
	}

	/* Step 2: Enable TouchInfo in data output (DataOutputEnableMask = 0x00A0) */
	ret = mgc3130_set_runtime_parameter(
		MGC3130_PARAM_DATA_OUTPUT_ENABLE,
		MGC3130_OUTPUT_TOUCH_INFO,  /* arg0: 0x00000002 = TouchInfo field */
		0xFFFFFFFF                   /* arg1: 0xFFFFFFFF = overwrite mask */
	);
	if (ret < 0) {
		LOG_ERR("Failed to enable TouchInfo output: %d", ret);
		return ret;
	}

	/* Update configuration state */
	mgc_inst.config.touch_enabled = true;
	mgc_inst.config.output_mask = MGC3130_OUTPUT_TOUCH_INFO;
	mgc_inst.configured = true;

	/* Initialize touch state */
	mgc_inst.touch_state.current_touch = 0;
	mgc_inst.touch_state.previous_touch = 0;
	mgc_inst.touch_state.touch_active = false;
	mgc_inst.touch_state.touch_start_time = 0;

	LOG_INF("MGC3130 touch detection configured successfully");
	return 0;
}

/*
 * Read and parse Sensor_Data_Output message
 *
 * @param touch_info: Output pointer for touch information (can be NULL)
 * @return: 0 on success, negative error code on failure
 */
static int mgc3130_read_sensor_data(mgc3130_touch_info_t *touch_info)
{
	uint8_t buf[MGC3130_MSG_BUF_SIZE];
	int ret;
	uint16_t msg_size;
	uint8_t msg_id;
	uint16_t config_mask;
	size_t offset;

	/* Read message from sensor */
	ret = mgc3130_read_message(buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	msg_size = buf[0] | (buf[1] << 8);
	msg_id = buf[3];

	/* Validate message ID */
	if (msg_id != MGC3130_MSG_SENSOR_DATA_OUTPUT) {
		LOG_WRN("Unexpected message ID: 0x%02X (expected 0x91)", msg_id);
		return -EINVAL;
	}

	/* Parse fixed header: size(2) + flags(1) + id(1) = 4 bytes minimum */
	offset = 4;

	/* Parse config mask (2 bytes) */
	if (offset + 2 > msg_size) {
		LOG_ERR("Message too short for config mask");
		return -EINVAL;
	}
	config_mask = buf[offset] | (buf[offset + 1] << 8);
	offset += 2;

	/* Skip timestamp (1 byte) and system_info (1 byte) */
	offset += 2;

	/* Parse variable fields based on config_mask */

	/* TouchInfo field (4 bytes) - bit 2 */
	if ((config_mask & MGC3130_OUTPUT_TOUCH_INFO) && touch_info != NULL) {
		if (offset + 4 > msg_size) {
			LOG_ERR("Message too short for TouchInfo field");
			return -EINVAL;
		}

		touch_info->touch_flags = buf[offset] | (buf[offset + 1] << 8);
		touch_info->touch_counter = buf[offset + 2];
		touch_info->reserved = buf[offset + 3];
		offset += 4;
	}

	LOG_DBG("Sensor data parsed: config_mask=0x%04X, size=%u", config_mask, msg_size);
	return 0;
}

/*
 * Process touch state transitions and log touch cycles
 * Detects and logs when touch starts (any electrode touched) and
 * when touch ends (all electrodes released)
 *
 * @param touch_info: Current touch information from sensor
 */
static void mgc3130_process_touch_state(const mgc3130_touch_info_t *touch_info)
{
	uint8_t touch_electrodes;
	uint32_t current_time;
	uint32_t touch_duration;

	/* Extract electrode touch bits (bits 0-3: S, W, N, E) */
	touch_electrodes = touch_info->touch_flags & 0x0F;

	/* Update current touch state */
	mgc_inst.touch_state.previous_touch = mgc_inst.touch_state.current_touch;
	mgc_inst.touch_state.current_touch = touch_electrodes;
	current_time = k_uptime_get_32();

	/* Detect touch cycle START: transition from no-touch to any-touch */
	if (!mgc_inst.touch_state.touch_active && touch_electrodes != 0) {
		/* Touch started */
		mgc_inst.touch_state.touch_active = true;
		mgc_inst.touch_state.touch_start_time = current_time;

		/* Log which electrodes are touched */
		LOG_INF("Touch START - Electrodes: %s%s%s%s (counter=%u)",
		        (touch_electrodes & MGC3130_TOUCH_SOUTH) ? "S" : "",
		        (touch_electrodes & MGC3130_TOUCH_WEST)  ? "W" : "",
		        (touch_electrodes & MGC3130_TOUCH_NORTH) ? "N" : "",
		        (touch_electrodes & MGC3130_TOUCH_EAST)  ? "E" : "",
		        touch_info->touch_counter);
	}
	/* Detect touch cycle END: transition from any-touch to no-touch */
	else if (mgc_inst.touch_state.touch_active && touch_electrodes == 0) {
		/* Touch ended */
		touch_duration = current_time - mgc_inst.touch_state.touch_start_time;
		mgc_inst.touch_state.touch_active = false;

		/* Log end of touch with duration */
		LOG_INF("Touch END - Duration: %u ms, Last electrodes: %s%s%s%s",
		        touch_duration,
		        (mgc_inst.touch_state.previous_touch & MGC3130_TOUCH_SOUTH) ? "S" : "",
		        (mgc_inst.touch_state.previous_touch & MGC3130_TOUCH_WEST)  ? "W" : "",
		        (mgc_inst.touch_state.previous_touch & MGC3130_TOUCH_NORTH) ? "N" : "",
		        (mgc_inst.touch_state.previous_touch & MGC3130_TOUCH_EAST)  ? "E" : "");
	}
	/* Touch ongoing - detect electrode changes */
	else if (mgc_inst.touch_state.touch_active &&
	         touch_electrodes != mgc_inst.touch_state.previous_touch) {
		/* Electrode configuration changed during touch */
		LOG_DBG("Touch change - Electrodes: %s%s%s%s",
		        (touch_electrodes & MGC3130_TOUCH_SOUTH) ? "S" : "",
		        (touch_electrodes & MGC3130_TOUCH_WEST)  ? "W" : "",
		        (touch_electrodes & MGC3130_TOUCH_NORTH) ? "N" : "",
		        (touch_electrodes & MGC3130_TOUCH_EAST)  ? "E" : "");
	}
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
	int ts_value;
	mgc3130_touch_info_t touch_info;

	if (mgc_inst.base.state != DRIVER_STATE_OPEN) {
		LOG_ERR("Driver not open");
		return -EINVAL;
	}

	DRIVER_LOCK(&mgc_inst.base);

	/* Non-blocking check: Read TS pin */
	ts_value = gpio_pin_get(mgc_inst.gpio_dev, MGC3130_TS_PIN);
	if (ts_value < 0) {
		LOG_ERR("Failed to read TS pin: %d", ts_value);
		DRIVER_UNLOCK(&mgc_inst.base);
		return ts_value;
	}

	/* If TS is high - no data available */
	if (ts_value != 0) {
		DRIVER_UNLOCK(&mgc_inst.base);
		return DRIVER_ERR_AGAIN;
	}

	/* TS is low - data ready, read sensor data */
	ret = mgc3130_read_sensor_data(&touch_info);
	if (ret < 0) {
		LOG_WRN("Failed to read sensor data: %d", ret);
		DRIVER_UNLOCK(&mgc_inst.base);
		return ret;
	}

	/* Process touch state transitions and log */
	mgc3130_process_touch_state(&touch_info);

	/* Copy touch_info to user buffer if provided */
	if (buf != NULL && count >= sizeof(mgc3130_touch_info_t)) {
		memcpy(buf, &touch_info, sizeof(mgc3130_touch_info_t));
	}

	DRIVER_UNLOCK(&mgc_inst.base);

	return sizeof(mgc3130_touch_info_t);
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

	case MGC_IOCTL_CONFIGURE_TOUCH:
		/* Manually trigger touch configuration */
		ret = mgc3130_configure_touch();
		break;

	case MGC_IOCTL_READ_SENSOR_DATA:
		/* Single sensor data read */
		if (arg == NULL) {
			ret = -EINVAL;
			break;
		}
		ret = mgc3130_read_sensor_data((mgc3130_touch_info_t *)arg);
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

	/* Configure touch detection */
	ret = mgc3130_configure_touch();
	if (ret < 0) {
		LOG_ERR("Failed to configure touch detection: %d", ret);
		mgc_inst.configured = false;
		/* Don't fail initialization - allow manual configuration via ioctl */
	} else {
		mgc_inst.configured = true;
	}

	/* State remains DRIVER_STATE_CLOSED - application must call open() */

	LOG_INF("MGC3130 handler initialized successfully");

	return 0;
}

SYS_INIT(mgc_init, APPLICATION, 92);
