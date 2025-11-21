/*
 * Central Thread Implementation
 *
 * Orchestrates all sensor reading, communication, and output control
 */

#include "central_thread.h"
#include "comm/esb_handler.h"
#include "input/btn_handler.h"
#include "sensors/imu_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(central_thread, LOG_LEVEL_INF);

#define CENTRAL_THREAD_STACK_SIZE 2048
#define CENTRAL_THREAD_PRIORITY 5
#define CENTRAL_THREAD_PERIOD_MS 10

/* Packet sequence number */
static uint32_t packet_sequence = 0;

/* Central thread entry point */
static void central_thread_entry(void *p1, void *p2, void *p3)
{
	int err;
	sensor_data_t sensor_packet;
	uint8_t rx_buf[32];
	size_t rx_len;

	LOG_INF("Central thread started");

	while (1) {
		/* Read and log IMU sensor data */
		imu_data_t imu_data;
		err = imu_handler_read(&imu_data);
		if (err) {
			LOG_WRN("IMU read failed: %d", err);
		} else {
			LOG_INF("IMU: Accel[%d, %d, %d] Gyro[%d, %d, %d]",
				imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
				imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
		}

		/* Get button state */
		uint8_t btn_state = btn_handler_get_state();

		/* Build sensor data packet */
		sensor_packet.sequence_num = packet_sequence++;
		sensor_packet.btn_state = btn_state;
		sensor_packet.timestamp_ms = k_uptime_get_32();

		/* Send sensor data via ESB */
		err = esb_handler_send(&sensor_packet);
		if (err) {
			LOG_ERR("ESB send failed: %d", err);
		}

		/* Check for received ESB data */
		rx_len = sizeof(rx_buf);
		err = esb_handler_receive(rx_buf, &rx_len);
		if (err == 0 && rx_len > 0) {
			/* Process received data */
			LOG_INF("Received ESB data, len: %zu", rx_len);
			LOG_HEXDUMP_INF(rx_buf, rx_len, "RX:");
			/* TODO: Parse and handle LED/vibration commands when implemented */
		}

		/* Sleep for 10ms (100Hz) */
		k_sleep(K_MSEC(CENTRAL_THREAD_PERIOD_MS));
	}
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
