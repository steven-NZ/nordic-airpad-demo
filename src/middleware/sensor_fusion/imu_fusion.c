/**
 * IMU Sensor Fusion Implementation
 *
 * 6-DOF RTQF (Real-Time Quaternion Filter) algorithm
 */

#include "imu_fusion.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Default configuration values */
#define DEFAULT_SLERP_POWER     0.02f    /* 2% correction per sample */
#define DEFAULT_SAMPLE_RATE_HZ  100      /* 100 Hz */

/* Limits for error checking */
#define MAX_DT_SECONDS          0.1f     /* 100ms - reject larger dt as error */
#define MIN_DT_SECONDS          0.0001f  /* 0.1ms - reject smaller dt as error */

/**
 * Internal fusion state structure
 * (Opaque to users - only accessible through API)
 */
struct imu_fusion_state {
    /* Configuration */
    imu_fusion_config_t config;

    /* Algorithm state */
    quaternion_t state_q;           /* Current predicted orientation quaternion */
    quaternion_t measured_q;        /* Measured orientation from accel */
    euler_angles_t measured_euler;  /* Measured orientation as Euler */

    /* Timing */
    uint64_t last_timestamp_us;     /* Previous sample timestamp */
    imu_float_t dt;                 /* Delta time in seconds */

    /* Internal working variables (for SLERP update) */
    quaternion_t rotation_delta;    /* Rotation from predicted to measured */
    quaternion_t rotation_power;    /* SLERP result */
    vector3_t rotation_unit_vec;    /* Axis of rotation */

    /* Output */
    imu_fusion_output_t output;     /* Latest output */

    /* Initialization */
    bool first_update;              /* True until first sample processed */
    uint32_t sample_count;          /* Number of samples processed */
};

/* ========== Forward declarations ========== */
static void fusion_predict(imu_fusion_state_t *state, const vector3_t *gyro);
static void fusion_update(imu_fusion_state_t *state);
static void calculate_measured_pose(imu_fusion_state_t *state, const vector3_t *accel);
static bool validate_sensor_data(const vector3_t *accel, const vector3_t *gyro);

/* ========== Public API Implementation ========== */

imu_fusion_state_t *imu_fusion_init(const imu_fusion_config_t *config)
{
    imu_fusion_state_t *state = (imu_fusion_state_t *)malloc(sizeof(imu_fusion_state_t));
    if (!state) {
        return NULL;
    }

    memset(state, 0, sizeof(imu_fusion_state_t));

    /* Set configuration (use defaults if NULL) */
    if (config) {
        state->config = *config;
    } else {
        state->config.slerp_power = DEFAULT_SLERP_POWER;
        state->config.enable_gyro = true;
        state->config.enable_accel = true;
        state->config.sample_rate_hz = DEFAULT_SAMPLE_RATE_HZ;
    }

    /* Clamp SLERP power to valid range */
    if (state->config.slerp_power < 0.0f) {
        state->config.slerp_power = 0.0f;
    } else if (state->config.slerp_power > 1.0f) {
        state->config.slerp_power = 1.0f;
    }

    /* Initialize quaternion to identity (no rotation) */
    quat_identity(&state->state_q);
    quat_identity(&state->measured_q);

    /* Mark as uninitialized */
    state->first_update = true;
    state->output.valid = false;

    return state;
}

void imu_fusion_reset(imu_fusion_state_t *state)
{
    if (!state) {
        return;
    }

    /* Reset state quaternion to identity */
    quat_identity(&state->state_q);
    quat_identity(&state->measured_q);

    /* Reset timing */
    state->last_timestamp_us = 0;
    state->dt = 0.0f;

    /* Mark as uninitialized */
    state->first_update = true;
    state->sample_count = 0;
    state->output.valid = false;
}

int imu_fusion_update(imu_fusion_state_t *state,
                      const vector3_t *accel,
                      const vector3_t *gyro,
                      uint64_t timestamp_us,
                      imu_fusion_output_t *output)
{
    /* Validate parameters */
    if (!state || !accel || !gyro) {
        return -1;  /* Invalid parameters */
    }

    /* Validate sensor data (check for NaN/infinity) */
    if (!validate_sensor_data(accel, gyro)) {
        return -3;  /* Invalid sensor data */
    }

    /* First update - initialize from accelerometer */
    if (state->first_update) {
        /* Calculate initial orientation from gravity direction */
        vector3_accel_to_euler(accel, &state->measured_euler);

        /* Convert to quaternion */
        quat_from_euler(&state->measured_euler, &state->state_q);

        /* Store timestamp */
        state->last_timestamp_us = timestamp_us;

        /* Mark as initialized */
        state->first_update = false;
        state->sample_count = 0;

        /* Generate initial output */
        state->output.orientation = state->state_q;
        quat_to_euler(&state->state_q, &state->output.euler);
        state->output.timestamp_us = timestamp_us;
        state->output.valid = true;

        if (output) {
            *output = state->output;
        }

        return 0;
    }

    /* Calculate delta time */
    if (timestamp_us <= state->last_timestamp_us) {
        return -2;  /* Invalid timestamp (backwards in time) */
    }

    state->dt = (imu_float_t)(timestamp_us - state->last_timestamp_us) / 1000000.0f;

    /* Validate dt */
    if (state->dt > MAX_DT_SECONDS || state->dt < MIN_DT_SECONDS) {
        return -2;  /* Invalid dt (too large or too small) */
    }

    state->last_timestamp_us = timestamp_us;

    /* ===== Predict-Update Cycle ===== */

    /* 1. Predict step: Integrate gyroscope angular velocity */
    if (state->config.enable_gyro) {
        fusion_predict(state, gyro);
    }

    /* 2. Measurement step: Calculate orientation from accelerometer */
    if (state->config.enable_accel) {
        calculate_measured_pose(state, accel);
    }

    /* 3. Update step: Correct prediction with measurement using SLERP */
    if (state->config.enable_accel) {
        fusion_update(state);
    }

    /* 4. Generate output */
    state->output.orientation = state->state_q;
    quat_to_euler(&state->state_q, &state->output.euler);
    state->output.timestamp_us = timestamp_us;
    state->output.valid = true;

    state->sample_count++;

    if (output) {
        *output = state->output;
    }

    return 0;
}

void imu_fusion_get_output(const imu_fusion_state_t *state,
                           imu_fusion_output_t *output)
{
    if (!state || !output) {
        return;
    }

    *output = state->output;
}

void imu_fusion_set_slerp_power(imu_fusion_state_t *state, imu_float_t power)
{
    if (!state) {
        return;
    }

    /* Clamp to valid range */
    if (power < 0.0f) {
        power = 0.0f;
    } else if (power > 1.0f) {
        power = 1.0f;
    }

    state->config.slerp_power = power;
}

imu_float_t imu_fusion_get_slerp_power(const imu_fusion_state_t *state)
{
    if (!state) {
        return 0.0f;
    }

    return state->config.slerp_power;
}

void imu_fusion_destroy(imu_fusion_state_t *state)
{
    if (state) {
        free(state);
    }
}

/* ========== Internal Algorithm Functions ========== */

/**
 * Predict step: Integrate gyroscope angular velocity */
static void fusion_predict(imu_fusion_state_t *state, const vector3_t *gyro)
{
    /* Get current quaternion components */
    imu_float_t qw = state->state_q.w;
    imu_float_t qx = state->state_q.x;
    imu_float_t qy = state->state_q.y;
    imu_float_t qz = state->state_q.z;

    /* Gyro angular velocity divided by 2 (quaternion kinematic equation) */
    imu_float_t gx2 = gyro->x / 2.0f;
    imu_float_t gy2 = gyro->y / 2.0f;
    imu_float_t gz2 = gyro->z / 2.0f;

    /* Quaternion derivative: dq/dt = 0.5 * omega * q
     * Integrate using Euler method: q_new = q_old + dq/dt * dt
     */
    state->state_q.w += (-gx2 * qx - gy2 * qy - gz2 * qz) * state->dt;
    state->state_q.x += (gx2 * qw + gz2 * qy - gy2 * qz) * state->dt;
    state->state_q.y += (gy2 * qw - gz2 * qx + gx2 * qz) * state->dt;
    state->state_q.z += (gz2 * qw + gy2 * qx - gx2 * qy) * state->dt;

    /* Normalize to prevent drift */
    quat_normalize(&state->state_q);
}

/**
 * Calculate measured orientation from accelerometer
 */
static void calculate_measured_pose(imu_fusion_state_t *state, const vector3_t *accel)
{
    /* Convert accelerometer to Euler angles (tilt from gravity) */
    vector3_accel_to_euler(accel, &state->measured_euler);

    /* No magnetometer - cannot determine absolute yaw
     * Keep previous yaw from prediction
     */
    state->measured_euler.yaw = state->output.euler.yaw;

    /* Convert Euler to quaternion */
    quat_from_euler(&state->measured_euler, &state->measured_q);

    /* Handle quaternion aliasing (double cover property)
     * Ensure measured_q and state_q have consistent signs
     */
    int max_index = 0;
    imu_float_t max_val = fabsf(state->measured_q.w);

    if (fabsf(state->measured_q.x) > max_val) {
        max_val = fabsf(state->measured_q.x);
        max_index = 1;
    }
    if (fabsf(state->measured_q.y) > max_val) {
        max_val = fabsf(state->measured_q.y);
        max_index = 2;
    }
    if (fabsf(state->measured_q.z) > max_val) {
        max_val = fabsf(state->measured_q.z);
        max_index = 3;
    }

    /* Get corresponding components */
    imu_float_t measured_val, state_val;
    switch (max_index) {
        case 0: measured_val = state->measured_q.w; state_val = state->state_q.w; break;
        case 1: measured_val = state->measured_q.x; state_val = state->state_q.x; break;
        case 2: measured_val = state->measured_q.y; state_val = state->state_q.y; break;
        case 3: measured_val = state->measured_q.z; state_val = state->state_q.z; break;
        default: measured_val = state->measured_q.w; state_val = state->state_q.w; break;
    }

    /* If signs differ, flip measured quaternion */
    if ((measured_val < 0.0f && state_val > 0.0f) ||
        (measured_val > 0.0f && state_val < 0.0f)) {
        state->measured_q.w = -state->measured_q.w;
        state->measured_q.x = -state->measured_q.x;
        state->measured_q.y = -state->measured_q.y;
        state->measured_q.z = -state->measured_q.z;
    }
}

/**
 * Update step: Correct prediction using SLERP */
static void fusion_update(imu_fusion_state_t *state)
{
    /* Calculate rotation delta from predicted to measured
     * delta = state_q^(-1) * measured_q = state_q* * measured_q
     */
    quaternion_t state_conj;
    quat_conjugate(&state->state_q, &state_conj);
    quat_multiply(&state_conj, &state->measured_q, &state->rotation_delta);
    quat_normalize(&state->rotation_delta);

    /* SLERP: Spherical Linear Interpolation
     * Interpolate from identity to rotation_delta by slerp_power amount
     */
    imu_float_t theta = acosf(state->rotation_delta.w);

    /* Clamp theta to prevent numerical issues */
    if (theta > IMU_PI) {
        theta = IMU_PI;
    }

    imu_float_t sin_theta = sinf(theta);

    /* Extract rotation axis (handle near-zero rotation) */
    if (fabsf(sin_theta) > 0.001f) {
        imu_float_t sin_theta_inv = 1.0f / sin_theta;
        state->rotation_unit_vec.x = state->rotation_delta.x * sin_theta_inv;
        state->rotation_unit_vec.y = state->rotation_delta.y * sin_theta_inv;
        state->rotation_unit_vec.z = state->rotation_delta.z * sin_theta_inv;
    } else {
        /* Near-zero rotation - use vector part directly */
        state->rotation_unit_vec.x = state->rotation_delta.x;
        state->rotation_unit_vec.y = state->rotation_delta.y;
        state->rotation_unit_vec.z = state->rotation_delta.z;
    }
    vector3_normalize(&state->rotation_unit_vec);

    /* Build SLERP'd quaternion at power fraction of original rotation */
    imu_float_t theta_power = theta * state->config.slerp_power;
    imu_float_t sin_theta_power = sinf(theta_power);
    imu_float_t cos_theta_power = cosf(theta_power);

    state->rotation_power.w = cos_theta_power;
    state->rotation_power.x = sin_theta_power * state->rotation_unit_vec.x;
    state->rotation_power.y = sin_theta_power * state->rotation_unit_vec.y;
    state->rotation_power.z = sin_theta_power * state->rotation_unit_vec.z;
    quat_normalize(&state->rotation_power);

    /* Apply correction: state_q = state_q * rotation_power */
    quaternion_t corrected_q;
    quat_multiply(&state->state_q, &state->rotation_power, &corrected_q);
    state->state_q = corrected_q;

    /* Final normalization */
    quat_normalize(&state->state_q);
}

/**
 * Validate sensor data (check for NaN and infinity)
 */
static bool validate_sensor_data(const vector3_t *accel, const vector3_t *gyro)
{
    /* Check accelerometer */
    if (isnan(accel->x) || isnan(accel->y) || isnan(accel->z) ||
        isinf(accel->x) || isinf(accel->y) || isinf(accel->z)) {
        return false;
    }

    /* Check gyroscope */
    if (isnan(gyro->x) || isnan(gyro->y) || isnan(gyro->z) ||
        isinf(gyro->x) || isinf(gyro->y) || isinf(gyro->z)) {
        return false;
    }

    return true;
}
