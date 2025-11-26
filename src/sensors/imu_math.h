/**
 * IMU Math Library
 *
 * Quaternion and vector mathematics for IMU sensor fusion.
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>
#include <stdbool.h>

/* Use float for embedded efficiency */
typedef float imu_float_t;

/* Mathematical constants */
#define IMU_PI              3.14159265358979f
#define IMU_DEG_TO_RAD      (IMU_PI / 180.0f)
#define IMU_RAD_TO_DEG      (180.0f / IMU_PI)

/**
 * 3D Vector
 * Used for accelerometer, gyroscope data, and Euler angles
 */
typedef struct {
    imu_float_t x;
    imu_float_t y;
    imu_float_t z;
} vector3_t;

/**
 * Quaternion
 * Represents 3D rotation: q = w + xi + yj + zk
 * w is the scalar (real) component
 * x, y, z are the vector (imaginary) components
 */
typedef struct {
    imu_float_t w;  /* scalar component */
    imu_float_t x;  /* i component */
    imu_float_t y;  /* j component */
    imu_float_t z;  /* k component */
} quaternion_t;

/**
 * Euler Angles
 * Represents orientation as roll, pitch, yaw (in radians)
 * Note: Euler angles have gimbal lock at ±90° pitch
 */
typedef struct {
    imu_float_t roll;   /* rotation around X axis (radians) */
    imu_float_t pitch;  /* rotation around Y axis (radians) */
    imu_float_t yaw;    /* rotation around Z axis (radians) */
} euler_angles_t;

/* ========== Vector3 Operations ========== */

/**
 * Initialize vector to zero
 */
void vector3_zero(vector3_t *v);

/**
 * Set vector components
 */
void vector3_set(vector3_t *v, imu_float_t x, imu_float_t y, imu_float_t z);

/**
 * Calculate vector length (magnitude)
 */
imu_float_t vector3_length(const vector3_t *v);

/**
 * Normalize vector to unit length
 * If vector has zero length, no operation is performed
 */
void vector3_normalize(vector3_t *v);

/**
 * Calculate dot product of two vectors
 * Returns: a · b
 */
imu_float_t vector3_dot(const vector3_t *a, const vector3_t *b);

/**
 * Calculate cross product of two vectors
 * result = a × b
 */
void vector3_cross(const vector3_t *a, const vector3_t *b, vector3_t *result);

/**
 * Convert accelerometer reading to Euler angles
 *
 * Extracts roll and pitch from normalized accelerometer vector
 * (assumes accelerometer measures gravity direction)
 * Yaw is set to 0 (cannot be determined from accelerometer alone)
 *
 * @param accel Accelerometer vector (will be normalized internally)
 * @param euler Output Euler angles (radians)
 */
void vector3_accel_to_euler(const vector3_t *accel, euler_angles_t *euler);

/* ========== Quaternion Operations ========== */

/**
 * Initialize quaternion to zero
 */
void quat_zero(quaternion_t *q);

/**
 * Initialize quaternion to identity (no rotation)
 * Identity quaternion: (1, 0, 0, 0)
 */
void quat_identity(quaternion_t *q);

/**
 * Set quaternion components
 */
void quat_set(quaternion_t *q, imu_float_t w, imu_float_t x,
              imu_float_t y, imu_float_t z);

/**
 * Normalize quaternion to unit length
 * Critical for preventing drift in fusion algorithms
 * If quaternion has zero or unit length, no operation is performed
 */
void quat_normalize(quaternion_t *q);

/**
 * Multiply two quaternions: result = a * b
 *
 * Quaternion multiplication is NOT commutative: a*b ≠ b*a
 * This represents applying rotation b, then rotation a
 *
 * @param a Left operand
 * @param b Right operand
 * @param result Output quaternion (can be same as a or b)
 */
void quat_multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *result);

/**
 * Calculate quaternion conjugate
 *
 * Conjugate of q = (w, x, y, z) is (w, -x, -y, -z)
 * For unit quaternions, conjugate equals inverse
 * Used for inverse rotations
 *
 * @param q Input quaternion
 * @param result Output conjugate (can be same as q)
 */
void quat_conjugate(const quaternion_t *q, quaternion_t *result);

/* ========== Quaternion <-> Euler Conversions ========== */

/**
 * Convert quaternion to Euler angles
 *
 * Converts quaternion orientation to roll, pitch, yaw
 * Uses standard aerospace sequence (ZYX)
 *
 * WARNING: Euler angles have singularity at ±90° pitch (gimbal lock)
 * Use quaternions as primary representation when possible
 *
 * @param q Input quaternion
 * @param euler Output Euler angles (radians)
 */
void quat_to_euler(const quaternion_t *q, euler_angles_t *euler);

/**
 * Convert Euler angles to quaternion
 *
 * @param euler Input Euler angles (radians)
 * @param q Output quaternion (normalized)
 */
void quat_from_euler(const euler_angles_t *euler, quaternion_t *q);

#endif /* IMU_MATH_H */
