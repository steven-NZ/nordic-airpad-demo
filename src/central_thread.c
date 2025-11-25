/*
 * Central Thread Implementation
 *
 * Orchestrates all sensor reading, communication, and output control
 */

#include "central_thread.h"
#include "comm/esb_handler.h"
#include "input/btn_handler.h"
#include "sensors/imu_handler.h"
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
static driver_fd_t esb_fd = DRIVER_FD_INVALID;

/* Packet sequence number */
static uint32_t packet_sequence = 0;

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

	esb_fd = esb_open(0);
	if (esb_fd < 0) {
		LOG_ERR("Failed to open ESB driver: %d", esb_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("ESB driver opened with fd=%d", esb_fd);

	LOG_INF("All drivers opened successfully");

	while (1) {
		/* Read IMU sensor data */
		imu_data_t imu_data;
		result = imu_read(imu_fd, &imu_data, sizeof(imu_data));
		if (result < 0) {
			LOG_WRN("IMU read failed: %d", (int)result);
		} else {
			LOG_INF("IMU: Accel[%d, %d, %d] Gyro[%d, %d, %d]",
				imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
				imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
		}

		/* Read button state */
		result = btn_read(btn_fd, &btn_state_raw, sizeof(btn_state_raw));
		if (result < 0) {
			LOG_WRN("Button read failed: %d", (int)result);
			btn_state_raw = 0;
		}

		/* Build sensor data packet */
		sensor_packet.sequence_num = packet_sequence++;
		sensor_packet.btn_state = btn_state_raw;
		sensor_packet.timestamp_ms = k_uptime_get_32();

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
	esb_close(esb_fd);
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
