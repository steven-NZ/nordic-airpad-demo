/*
 * IMU Handler for ICM-42670-P
 */

#include "imu_handler.h"
#include "../drivers/driver_framework.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <string.h>

LOG_MODULE_REGISTER(imu_handler, LOG_LEVEL_INF);

/*
 * Private Driver State
 */
typedef struct {
	driver_instance_t base;         /* Base driver instance (must be first) */
	const struct device *dev;       /* Zephyr device handle */
	imu_config_t config;            /* Current configuration */
	uint32_t read_count;            /* Statistics: read operations */
	uint32_t error_count;           /* Statistics: error count */
} imu_instance_t;

/* Single IMU instance (static allocation - only one IMU in system) */
static imu_instance_t imu_inst;
static driver_instance_t *imu_instances[1] = { &imu_inst.base };

/* Get IMU device from device tree */
static const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(icm42670p));

/*
 * Device initialization - called automatically at boot via SYS_INIT
 * Checks that the IMU device is ready before drivers are used
 */
static int imu_device_init(void)
{
	int retry_count = 0;
	const int max_retries = 5;

	/* Initialize driver instance */
	DRIVER_INSTANCE_INIT(&imu_inst.base);
	imu_inst.dev = imu_dev;
	imu_inst.read_count = 0;
	imu_inst.error_count = 0;

	/* Default configuration from device tree */
	imu_inst.config.accel_hz = 200;
	imu_inst.config.gyro_hz = 200;
	imu_inst.config.accel_range = 8;
	imu_inst.config.gyro_range = 1000;

	k_msleep(20);  /* Initial startup delay */

	/* Wait for device to be ready with retries */
	while (!device_is_ready(imu_dev) && retry_count < max_retries) {
		LOG_WRN("IMU device %s not ready, retrying... (%d/%d)",
			imu_dev->name, retry_count + 1, max_retries);
		k_msleep(10);
		retry_count++;
	}

	if (!device_is_ready(imu_dev)) {
		LOG_ERR("IMU device %s not ready after %d retries",
			imu_dev->name, max_retries);
		return -ENODEV;
	}

	LOG_INF("IMU device initialized: %s", imu_dev->name);
	return 0;
}

/* Register device initialization at boot (POST_KERNEL level, priority 90) */
SYS_INIT(imu_device_init, POST_KERNEL, 90);

/*
 * Open IMU driver - Returns file descriptor
 */
driver_fd_t imu_open(uint32_t flags)
{
	driver_fd_t fd;

	/* Only one IMU instance supported */
	if (imu_inst.base.state == DRIVER_STATE_OPEN) {
		LOG_WRN("IMU already open");
		return DRIVER_ERR_BUSY;
	}

	/* Verify device is still ready */
	if (!device_is_ready(imu_inst.dev)) {
		LOG_ERR("IMU device not ready");
		imu_inst.base.state = DRIVER_STATE_ERROR;
		return DRIVER_ERR_NODEV;
	}

	/* Allocate file descriptor (always returns 0 for single instance) */
	fd = driver_fd_alloc(imu_instances, 1);
	if (fd == DRIVER_FD_INVALID) {
		LOG_ERR("Failed to allocate IMU fd");
		return DRIVER_ERR_NOMEM;
	}

	/* Open driver instance */
	DRIVER_LOCK(&imu_inst.base);
	imu_inst.base.state = DRIVER_STATE_OPEN;
	imu_inst.base.flags = flags;
	DRIVER_UNLOCK(&imu_inst.base);

	LOG_INF("IMU opened with fd=%d", fd);
	return fd;
}

/*
 * Close IMU driver
 */
int imu_close(driver_fd_t fd)
{
	driver_instance_t *inst;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, imu_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid IMU fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Close driver instance */
	DRIVER_LOCK(inst);
	inst->state = DRIVER_STATE_CLOSED;
	inst->flags = 0;
	DRIVER_UNLOCK(inst);

	LOG_INF("IMU closed (fd=%d, reads=%u, errors=%u)",
		fd, imu_inst.read_count, imu_inst.error_count);

	return DRIVER_OK;
}

/*
 * Read IMU sensor data
 */
ssize_t imu_read(driver_fd_t fd, void *buf, size_t count)
{
	driver_instance_t *inst;
	imu_data_t *data = (imu_data_t *)buf;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int err;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, imu_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid IMU fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer */
	if (!buf || count < sizeof(imu_data_t)) {
		LOG_ERR("Invalid buffer (buf=%p, count=%zu)", buf, count);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	/* Fetch new sample from sensor */
	err = sensor_sample_fetch(imu_inst.dev);
	if (err) {
		LOG_ERR("Failed to fetch IMU sample: %d", err);
		imu_inst.error_count++;
		DRIVER_UNLOCK(inst);
		return DRIVER_ERR_IO;
	}

	/* Get accelerometer data (X, Y, Z) */
	err = sensor_channel_get(imu_inst.dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (err) {
		LOG_ERR("Failed to get accel data: %d", err);
		imu_inst.error_count++;
		DRIVER_UNLOCK(inst);
		return DRIVER_ERR_IO;
	}

	/* Get gyroscope data (X, Y, Z) */
	err = sensor_channel_get(imu_inst.dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (err) {
		LOG_ERR("Failed to get gyro data: %d", err);
		imu_inst.error_count++;
		DRIVER_UNLOCK(inst);
		return DRIVER_ERR_IO;
	}

	/* Convert sensor_value to int16_t
	 * Scale factor: multiply by 100 to preserve precision
	 * Accel: m/s² * 100 (e.g., 9.81 m/s² -> 981)
	 * Gyro: °/s * 100 (e.g., 45.5 °/s -> 4550)
	 */
	data->accel_x = (int16_t)(sensor_value_to_double(&accel[0]) * 100);
	data->accel_y = (int16_t)(sensor_value_to_double(&accel[1]) * 100);
	data->accel_z = (int16_t)(sensor_value_to_double(&accel[2]) * 100);
	data->gyro_x = (int16_t)(sensor_value_to_double(&gyro[0]) * 100);
	data->gyro_y = (int16_t)(sensor_value_to_double(&gyro[1]) * 100);
	data->gyro_z = (int16_t)(sensor_value_to_double(&gyro[2]) * 100);

	imu_inst.read_count++;

	DRIVER_UNLOCK(inst);

	return sizeof(imu_data_t);
}

/*
 * IMU driver control (ioctl)
 */
int imu_ioctl(driver_fd_t fd, unsigned int cmd, void *arg)
{
	driver_instance_t *inst;
	int err = DRIVER_OK;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, imu_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid IMU fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	DRIVER_LOCK(inst);

	switch (cmd) {
	case DRIVER_IOCTL_GET_STATUS:
		/* Return driver state */
		if (arg) {
			*(driver_state_t *)arg = inst->state;
		}
		break;

	case DRIVER_IOCTL_GET_INFO:
		/* Return device name */
		if (arg) {
			*(const char **)arg = imu_inst.dev->name;
		}
		break;

	case IMU_IOCTL_GET_SAMPLE_RATE:
		/* Return current sample rate configuration */
		if (arg) {
			imu_config_t *config = (imu_config_t *)arg;
			config->accel_hz = imu_inst.config.accel_hz;
			config->gyro_hz = imu_inst.config.gyro_hz;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case IMU_IOCTL_GET_ACCEL_RANGE:
		/* Return accelerometer range */
		if (arg) {
			*(uint16_t *)arg = imu_inst.config.accel_range;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case IMU_IOCTL_GET_GYRO_RANGE:
		/* Return gyroscope range */
		if (arg) {
			*(uint16_t *)arg = imu_inst.config.gyro_range;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case IMU_IOCTL_SET_SAMPLE_RATE:
	case IMU_IOCTL_SET_ACCEL_RANGE:
	case IMU_IOCTL_SET_GYRO_RANGE:
		/* Configuration changes not implemented (requires sensor driver support) */
		LOG_WRN("IMU configuration changes not supported (cmd=0x%x)", cmd);
		err = DRIVER_ERR_NOTSUP;
		break;

	default:
		LOG_WRN("Unknown IMU ioctl command: 0x%x", cmd);
		err = DRIVER_ERR_INVAL;
		break;
	}

	DRIVER_UNLOCK(inst);

	return err;
}
