/*
 * IMU Handler for ICM-42670-P
 */

#ifndef IMU_HANDLER_H_
#define IMU_HANDLER_H_

#include <stdint.h>
#include "../drivers/driver_framework.h"

/* IMU sensor data structure
 * All values in SI units:
 * - Accel: m/s²
 * - Gyro: rad/s
 */
typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} __attribute__((packed)) imu_data_t;

/* IMU-specific ioctl commands */
#define IMU_IOCTL_GET_SAMPLE_RATE   0x1001  /* Get sampling rate */
#define IMU_IOCTL_SET_SAMPLE_RATE   0x1002  /* Set sampling rate */
#define IMU_IOCTL_GET_ACCEL_RANGE   0x1003  /* Get accel range (G) */
#define IMU_IOCTL_SET_ACCEL_RANGE   0x1004  /* Set accel range */
#define IMU_IOCTL_GET_GYRO_RANGE    0x1005  /* Get gyro range (dps) */
#define IMU_IOCTL_SET_GYRO_RANGE    0x1006  /* Set gyro range */

/* IMU configuration structure for ioctl */
typedef struct {
	uint32_t accel_hz;      /* Accelerometer sample rate (Hz) */
	uint32_t gyro_hz;       /* Gyroscope sample rate (Hz) */
	uint16_t accel_range;   /* Accel range in G (2, 4, 8, 16) */
	uint16_t gyro_range;    /* Gyro range in dps (250, 500, 1000, 2000) */
} imu_config_t;

/**
 * @brief Open the IMU driver
 *
 * Initializes the ICM-42670-P sensor device and returns a file descriptor
 * for accessing the driver. Multiple instances are not supported (only one IMU).
 *
 * @param flags Open flags (reserved for future use, pass 0)
 * @return File descriptor on success, DRIVER_FD_INVALID on failure
 */
driver_fd_t imu_open(uint32_t flags);

/**
 * @brief Close the IMU driver
 *
 * Releases resources associated with the IMU driver instance.
 *
 * @param fd File descriptor returned by imu_open()
 * @return 0 on success, negative errno code on failure
 */
int imu_close(driver_fd_t fd);

/**
 * @brief Read IMU sensor data
 *
 * Fetches the latest accelerometer and gyroscope data from the IMU
 * and stores it in the provided buffer. The buffer must be at least
 * sizeof(imu_data_t) bytes.
 *
 * @param fd File descriptor returned by imu_open()
 * @param buf Pointer to buffer to store sensor readings
 * @param count Size of buffer (must be >= sizeof(imu_data_t))
 * @return Number of bytes read on success, negative errno code on failure
 */
ssize_t imu_read(driver_fd_t fd, void *buf, size_t count);

/**
 * @brief Control the IMU driver
 *
 * Performs control operations on the IMU driver such as configuring
 * sample rates, reading ranges, etc.
 *
 * @param fd File descriptor returned by imu_open()
 * @param cmd ioctl command (IMU_IOCTL_*)
 * @param arg Command-specific argument (depends on cmd)
 * @return 0 on success, negative errno code on failure
 */
int imu_ioctl(driver_fd_t fd, unsigned int cmd, void *arg);

#endif /* IMU_HANDLER_H_ */
