/*
 * IMU Handler for ICM-42670-P
 * Handles accelerometer and gyroscope data reading
 */

#ifndef IMU_HANDLER_H_
#define IMU_HANDLER_H_

#include <stdint.h>

/* IMU sensor data structure */
typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} __attribute__((packed)) imu_data_t;

/**
 * @brief Initialize the IMU handler
 *
 * Initializes the ICM-42670-P sensor device and verifies it's ready.
 *
 * @return 0 on success, negative errno code on failure
 */
int imu_handler_init(void);

/**
 * @brief Read IMU sensor data
 *
 * Fetches the latest accelerometer and gyroscope data from the IMU
 * and stores it in the provided data structure.
 *
 * @param data Pointer to imu_data_t structure to store sensor readings
 * @return 0 on success, negative errno code on failure
 */
int imu_handler_read(imu_data_t *data);

#endif /* IMU_HANDLER_H_ */
