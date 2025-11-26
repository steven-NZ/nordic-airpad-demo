/**
 * IMU Sensor Fusion API
 *
 * 6-DOF sensor fusion using RTQF (Real-Time Quaternion Filter) algorithm
 * Combines accelerometer and gyroscope data to produce orientation quaternion
 *
 * Based on RTIMULib RTQF implementation (richards-tech, LLC, 2014-2015)
 * Adapted for 6-DOF (no magnetometer)
 */

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "imu_math.h"
#include <stdint.h>
#include <stdbool.h>

/* Forward declaration of opaque state structure */
typedef struct imu_fusion_state imu_fusion_state_t;

/**
 * Fusion algorithm configuration
 */
typedef struct {
    imu_float_t slerp_power;    /* SLERP interpolation power (0.0-1.0)
                                 * Default: 0.02 (2% measurement influence)
                                 * Lower = more gyro trust, smoother but drifts
                                 * Higher = more accel trust, responsive but noisy */

    bool enable_gyro;            /* Enable gyroscope prediction (default: true) */
    bool enable_accel;           /* Enable accelerometer correction (default: true) */
    uint32_t sample_rate_hz;     /* Expected sample rate (default: 100 Hz) */
} imu_fusion_config_t;

/**
 * Fusion algorithm output
 */
typedef struct {
    quaternion_t orientation;    /* Current orientation as quaternion */
    euler_angles_t euler;        /* Current orientation as Euler angles (radians) */
    uint64_t timestamp_us;       /* Timestamp in microseconds */
    bool valid;                  /* True if fusion has been initialized */
} imu_fusion_output_t;

/**
 * Initialize fusion algorithm
 *
 * Allocates and initializes fusion state with given configuration.
 * The state starts uninitialized - first update will initialize from accelerometer.
 *
 * @param config Configuration parameters (NULL for defaults)
 * @return Pointer to fusion state, or NULL on failure
 */
imu_fusion_state_t *imu_fusion_init(const imu_fusion_config_t *config);

/**
 * Reset fusion to initial state
 *
 * Resets the fusion algorithm state while keeping configuration.
 * Use when detecting significant discontinuities or after sensor errors.
 *
 * @param state Fusion state
 */
void imu_fusion_reset(imu_fusion_state_t *state);

/**
 * Update fusion with new IMU sample
 *
 * Runs predict-update cycle:
 * 1. Predict orientation from gyroscope (angular velocity integration)
 * 2. Measure orientation from accelerometer (gravity direction)
 * 3. Correct prediction using SLERP with measured orientation
 *
 * On first call, initializes orientation from accelerometer only.
 *
 * @param state Fusion state
 * @param accel Accelerometer data in m/s² (raw or calibrated)
 * @param gyro Gyroscope data in rad/s (must be calibrated/zero-biased)
 * @param timestamp_us Timestamp in microseconds (monotonic)
 * @param output Pointer to store fusion result (optional, can be NULL)
 * @return 0 on success, negative error code on failure:
 *         -1: Invalid parameters (NULL pointers)
 *         -2: Invalid timestamp (backwards or too large dt)
 *         -3: Invalid sensor data (NaN or infinity)
 */
int imu_fusion_update(imu_fusion_state_t *state,
                      const vector3_t *accel,
                      const vector3_t *gyro,
                      uint64_t timestamp_us,
                      imu_fusion_output_t *output);

/**
 * Get current fusion output without new data
 *
 * Returns the most recent fusion result.
 * Useful for accessing current state between updates.
 *
 * @param state Fusion state
 * @param output Pointer to store fusion result
 */
void imu_fusion_get_output(const imu_fusion_state_t *state,
                           imu_fusion_output_t *output);

/**
 * Set SLERP power parameter
 *
 * Controls the influence of measurements vs. gyroscope predictions.
 * Valid range: 0.0 (gyro only) to 1.0 (measurements override gyro)
 *
 * Recommended values:
 * - 0.02 for 100 Hz sampling (default, good balance)
 * - 0.01 for 200 Hz sampling (more gyro trust)
 * - 0.05 for 50 Hz sampling (more accel trust)
 *
 * @param state Fusion state
 * @param power SLERP power (clamped to 0.0-1.0)
 */
void imu_fusion_set_slerp_power(imu_fusion_state_t *state, imu_float_t power);

/**
 * Get SLERP power parameter
 *
 * @param state Fusion state
 * @return Current SLERP power
 */
imu_float_t imu_fusion_get_slerp_power(const imu_fusion_state_t *state);

/**
 * Free fusion state resources
 *
 * Releases memory allocated by imu_fusion_init().
 * State pointer becomes invalid after this call.
 *
 * @param state Fusion state (can be NULL)
 */
void imu_fusion_destroy(imu_fusion_state_t *state);

#endif /* IMU_FUSION_H */
