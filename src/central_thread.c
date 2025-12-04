/*
 * Central Thread Implementation
 *
 * Orchestrates all sensor reading, communication, and output control
 */

#include "central_thread.h"
#include "comm/esb_handler.h"
#include "input/btn_handler.h"
#include "input/mgc_handler.h"
#include "sensors/imu_handler.h"
#include "sensors/imu_fusion.h"
#include "sensors/imu_math.h"
#include "drivers/driver_framework.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(central_thread, LOG_LEVEL_INF);

#define CENTRAL_THREAD_STACK_SIZE 4096
#define CENTRAL_THREAD_PRIORITY 5
#define CENTRAL_THREAD_PERIOD_MS 10

/* Driver file descriptors */
static driver_fd_t imu_fd = DRIVER_FD_INVALID;
static driver_fd_t btn_fd = DRIVER_FD_INVALID;
static driver_fd_t mgc_fd = DRIVER_FD_INVALID;
static driver_fd_t esb_fd = DRIVER_FD_INVALID;

/* IMU fusion state */
static imu_fusion_state_t *fusion_state = NULL;

/* Central thread entry point */
static void central_thread_entry(void *p1, void *p2, void *p3)
{
	ssize_t result;
	sensor_data_t sensor_packet;
	uint8_t rx_buf[32];
	uint8_t btn_state_raw;

	LOG_INF("Central thread started");

	/* Open all drivers */
	imu_fd = imu_open(0);
	if (imu_fd < 0) {
		LOG_ERR("Failed to open IMU driver: %d", imu_fd);
		return;
	}
	LOG_INF("IMU driver opened with fd=%d", imu_fd);

	btn_fd = btn_open(0);
	if (btn_fd < 0) {
		LOG_ERR("Failed to open button driver: %d", btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("Button driver opened with fd=%d", btn_fd);

	mgc_fd = mgc_open();
	if (mgc_fd < 0) {
		LOG_ERR("Failed to open MGC handler: %d", mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("MGC handler opened with fd=%d", mgc_fd);

	// /* Read MGC3130 firmware version */
	// mgc3130_fw_version_t mgc_fw_ver;
	// result = mgc_ioctl(mgc_fd, MGC_IOCTL_GET_FW_VERSION, &mgc_fw_ver);
	// if (result == 0) {
	// 	LOG_INF("MGC3130 FW Version: %s", mgc_fw_ver.version_string);
	// } else {
	// 	LOG_WRN("Failed to read MGC3130 FW version: %d", (int)result);
	// }

	esb_fd = esb_open(0);
	if (esb_fd < 0) {
		LOG_ERR("Failed to open ESB driver: %d", esb_fd);
		mgc_close(mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("ESB driver opened with fd=%d", esb_fd);

	/* Initialize IMU fusion */
	imu_fusion_config_t fusion_config = {
		.slerp_power = 0.02f,      /* 2% correction per sample */
		.enable_gyro = true,
		.enable_accel = true,
		.sample_rate_hz = 100
	};

	fusion_state = imu_fusion_init(&fusion_config);
	if (!fusion_state) {
		LOG_ERR("Failed to initialize IMU fusion");
		esb_close(esb_fd);
		mgc_close(mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("IMU fusion initialized (SLERP power: %.2f)",
	        (double)fusion_config.slerp_power);

	LOG_INF("All drivers opened successfully");

	while (1) {
		/* Read IMU sensor data */
		imu_data_t imu_data;
		imu_fusion_output_t fusion_output = {0};
		result = imu_read(imu_fd, &imu_data, sizeof(imu_data));
		if (result < 0) {
			LOG_WRN("IMU read failed: %d", (int)result);
		} else {
			/*  Accel: m/s², Gyro: rad/s */
			vector3_t accel, gyro;

			accel.x = imu_data.accel_x;
			accel.y = imu_data.accel_y;
			accel.z = imu_data.accel_z;

			gyro.x = imu_data.gyro_x;
			gyro.y = imu_data.gyro_y;
			gyro.z = imu_data.gyro_z;

			/* Get timestamp (convert ms to μs) */
			uint64_t timestamp_us = k_uptime_get() * 1000ULL;

			/* Update fusion */
			int ret = imu_fusion_update(fusion_state, &accel, &gyro,
			                            timestamp_us, &fusion_output);

			if (ret == 0 && fusion_output.valid) {
				/* Log every 10th sample (10Hz) to reduce verbosity */
				static uint32_t log_counter = 0;
				if (++log_counter >= 10) {
					log_counter = 0;
					LOG_INF("Quat[w:%.3f x:%.3f y:%.3f z:%.3f] "
					        "Euler[R:%.1f° P:%.1f° Y:%.1f°]",
					        (double)fusion_output.orientation.w,
					        (double)fusion_output.orientation.x,
					        (double)fusion_output.orientation.y,
					        (double)fusion_output.orientation.z,
					        (double)(fusion_output.euler.roll * IMU_RAD_TO_DEG),
					        (double)(fusion_output.euler.pitch * IMU_RAD_TO_DEG),
					        (double)(fusion_output.euler.yaw * IMU_RAD_TO_DEG));
				}
			} else {
				LOG_WRN("Fusion update failed: %d", ret);
			}
		}

		/* Read button state */
		result = btn_read(btn_fd, &btn_state_raw, sizeof(btn_state_raw));
		if (result < 0) {
			LOG_WRN("Button read failed: %d", (int)result);
			btn_state_raw = 0;
		}

		/* Poll MGC3130 touch sensor data */
		mgc3130_touch_info_t touch_info;
		result = mgc_read(mgc_fd, &touch_info, sizeof(touch_info));
		if (result < 0 && result != DRIVER_ERR_AGAIN) {
			LOG_WRN("MGC read failed: %d", (int)result);
		}
		/* Touch processing happens inside mgc_read() via state machine */

		/* Build sensor data packet with quaternion */
		sensor_packet.btn_state = btn_state_raw;

		/* Get latest quaternion from fusion output (already computed in lines 91-134) */
		if (fusion_output.valid) {
			/* Convert quaternion float to int16_t with scaling */
			sensor_packet.quat_w = QUAT_FLOAT_TO_INT16(fusion_output.orientation.w);
			sensor_packet.quat_x = QUAT_FLOAT_TO_INT16(fusion_output.orientation.x);
			sensor_packet.quat_y = QUAT_FLOAT_TO_INT16(fusion_output.orientation.y);
			sensor_packet.quat_z = QUAT_FLOAT_TO_INT16(fusion_output.orientation.z);
		} else {
			/* Fusion not ready - send identity quaternion (no rotation) */
			sensor_packet.quat_w = QUAT_FLOAT_TO_INT16(1.0f);
			sensor_packet.quat_x = 0;
			sensor_packet.quat_y = 0;
			sensor_packet.quat_z = 0;
		}

		/* Send sensor data via ESB */
		result = esb_write(esb_fd, &sensor_packet, sizeof(sensor_packet));
		if (result < 0) {
			LOG_ERR("ESB write failed: %d", (int)result);
		}

		/* Check for received ESB data */
		result = esb_read(esb_fd, rx_buf, sizeof(rx_buf));
		if (result > 0) {
			/* Process received data */
			LOG_INF("Received ESB data, len: %d", (int)result);
			LOG_HEXDUMP_INF(rx_buf, result, "RX:");
			/* TODO: Parse and handle LED/vibration commands when implemented */
		} else if (result < 0 && result != DRIVER_ERR_AGAIN) {
			/* Log errors other than "no data available" */
			LOG_WRN("ESB read error: %d", (int)result);
		}

		/* Sleep for 10ms (100Hz) */
		k_sleep(K_MSEC(CENTRAL_THREAD_PERIOD_MS));
	}

	/* Cleanup (never reached for now) */
	if (fusion_state) {
		imu_fusion_destroy(fusion_state);
	}
	esb_close(esb_fd);
	mgc_close(mgc_fd);
	btn_close(btn_fd);
	imu_close(imu_fd);
}

/* Define central thread */
K_THREAD_DEFINE(central_thread, CENTRAL_THREAD_STACK_SIZE,
		central_thread_entry, NULL, NULL, NULL,
		CENTRAL_THREAD_PRIORITY, 0, 0);

int central_thread_init(void)
{
	LOG_INF("Central thread initialized");
	return 0;
}
