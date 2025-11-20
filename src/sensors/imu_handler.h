/*
 * IMU Handler for ICM-42670-P
 * Handles accelerometer and gyroscope data reading
 */

#ifndef IMU_HANDLER_H_
#define IMU_HANDLER_H_

#include <stdint.h>

/**
 * @brief Initialize the IMU handler
 *
 * Initializes the ICM-42670-P sensor device and verifies it's ready.
 *
 * @return 0 on success, negative errno code on failure
 */
int imu_handler_init(void);

/**
 * @brief Read and log IMU sensor data
 *
 * Fetches the latest accelerometer and gyroscope data from the IMU
 * and logs it to the console in human-readable format (m/s² and °/s).
 *
 * This function is designed to be called from a timer callback (ISR context).
 *
 * @return 0 on success, negative errno code on failure
 */
int imu_handler_read(void);

#endif /* IMU_HANDLER_H_ */
