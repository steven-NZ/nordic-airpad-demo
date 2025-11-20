/*
 * IMU Handler for ICM-42670-P
 * Handles accelerometer and gyroscope data reading
 */

#include "imu_handler.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_handler, LOG_LEVEL_INF);

/* Get IMU device from device tree */
static const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(icm42670p));

int imu_handler_init(void)
{
	int retry_count = 0;
	const int max_retries = 5;

	k_msleep(100);

	/* Wait for device to be ready with retries */
	while (!device_is_ready(imu_dev) && retry_count < max_retries) {
		LOG_WRN("IMU device %s not ready, retrying... (%d/%d)",
			imu_dev->name, retry_count + 1, max_retries);
		k_msleep(10);  /* Wait 10ms before retry */
		retry_count++;
	}

	if (!device_is_ready(imu_dev)) {
		LOG_ERR("IMU device %s not ready after %d retries",
			imu_dev->name, max_retries);
		return -ENODEV;
	}

	LOG_INF("IMU handler initialized: %s", imu_dev->name);
	return 0;
}

int imu_handler_read(void)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int err;

	/* Fetch new sample from sensor */
	err = sensor_sample_fetch(imu_dev);
	if (err) {
		LOG_ERR("Failed to fetch IMU sample: %d", err);
		return err;
	}

	/* Get accelerometer data (X, Y, Z) */
	err = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (err) {
		LOG_ERR("Failed to get accel data: %d", err);
		return err;
	}

	/* Get gyroscope data (X, Y, Z) */
	err = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (err) {
		LOG_ERR("Failed to get gyro data: %d", err);
		return err;
	}

	/* Convert sensor_value to double and log
	 * Accel: m/s²
	 * Gyro: °/s (degrees per second)
	 */
	double ax = sensor_value_to_double(&accel[0]);
	double ay = sensor_value_to_double(&accel[1]);
	double az = sensor_value_to_double(&accel[2]);
	double gx = sensor_value_to_double(&gyro[0]);
	double gy = sensor_value_to_double(&gyro[1]);
	double gz = sensor_value_to_double(&gyro[2]);

	LOG_INF("IMU: Accel[%.2f, %.2f, %.2f] m/s² Gyro[%.2f, %.2f, %.2f] °/s",
		ax, ay, az, gx, gy, gz);

	return 0;
}
