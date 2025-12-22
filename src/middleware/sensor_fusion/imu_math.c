/**
 * IMU Math Library
 *
 * Quaternion and vector mathematics for IMU sensor fusion.
 */

#include "imu_math.h"
#include <math.h>
#include <string.h>

/* ========== Vector3 Operations ========== */

void vector3_zero(vector3_t *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

void vector3_set(vector3_t *v, imu_float_t x, imu_float_t y, imu_float_t z)
{
    v->x = x;
    v->y = y;
    v->z = z;
}

imu_float_t vector3_length(const vector3_t *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void vector3_normalize(vector3_t *v)
{
    imu_float_t length = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);

    if (length == 0.0f) {
        return;
    }

    v->x /= length;
    v->y /= length;
    v->z /= length;
}

imu_float_t vector3_dot(const vector3_t *a, const vector3_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector3_cross(const vector3_t *a, const vector3_t *b, vector3_t *result)
{
    /* Store in temporaries to handle result being same as input */
    imu_float_t x = a->y * b->z - a->z * b->y;
    imu_float_t y = a->z * b->x - a->x * b->z;
    imu_float_t z = a->x * b->y - a->y * b->x;

    result->x = x;
    result->y = y;
    result->z = z;
}

void vector3_accel_to_euler(const vector3_t *accel, euler_angles_t *euler)
{
    /* Normalize accelerometer vector */
    vector3_t norm_accel = *accel;
    vector3_normalize(&norm_accel);

    /* Extract roll and pitch from gravity direction */
    euler->roll = atan2f(norm_accel.y, norm_accel.z);
    euler->pitch = -atan2f(norm_accel.x,
                          sqrtf(norm_accel.y * norm_accel.y +
                                norm_accel.z * norm_accel.z));
    euler->yaw = 0.0f;  /* Cannot determine yaw from accelerometer alone */
}

/* ========== Quaternion Operations ========== */

void quat_zero(quaternion_t *q)
{
    q->w = 0.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quat_identity(quaternion_t *q)
{
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quat_set(quaternion_t *q, imu_float_t w, imu_float_t x,
              imu_float_t y, imu_float_t z)
{
    q->w = w;
    q->x = x;
    q->y = y;
    q->z = z;
}

void quat_normalize(quaternion_t *q)
{
    imu_float_t length = sqrtf(q->w * q->w + q->x * q->x +
                              q->y * q->y + q->z * q->z);

    if ((length == 0.0f) || (length == 1.0f)) {
        return;
    }

    q->w /= length;
    q->x /= length;
    q->y /= length;
    q->z /= length;
}

void quat_multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *result)
{
    /* Quaternion multiplication: result = a * b
     * Store in temporaries to handle result being same as input
     */
    imu_float_t w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    imu_float_t x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    imu_float_t y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    imu_float_t z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;

    result->w = w;
    result->x = x;
    result->y = y;
    result->z = z;
}

void quat_conjugate(const quaternion_t *q, quaternion_t *result)
{
    /* Conjugate: (w, -x, -y, -z) */
    result->w = q->w;
    result->x = -q->x;
    result->y = -q->y;
    result->z = -q->z;
}

/* ========== Quaternion <-> Euler Conversions ========== */

void quat_to_euler(const quaternion_t *q, euler_angles_t *euler)
{
    /* Convert quaternion to Euler angles (ZYX sequence) */
    euler->roll = atan2f(2.0f * (q->y * q->z + q->w * q->x),
                         1.0f - 2.0f * (q->x * q->x + q->y * q->y));

    euler->pitch = asinf(2.0f * (q->w * q->y - q->x * q->z));

    euler->yaw = atan2f(2.0f * (q->x * q->y + q->w * q->z),
                        1.0f - 2.0f * (q->y * q->y + q->z * q->z));
}

void quat_from_euler(const euler_angles_t *euler, quaternion_t *q)
{
    /* Convert Euler angles to quaternion */
    imu_float_t cos_x2 = cosf(euler->roll / 2.0f);
    imu_float_t sin_x2 = sinf(euler->roll / 2.0f);
    imu_float_t cos_y2 = cosf(euler->pitch / 2.0f);
    imu_float_t sin_y2 = sinf(euler->pitch / 2.0f);
    imu_float_t cos_z2 = cosf(euler->yaw / 2.0f);
    imu_float_t sin_z2 = sinf(euler->yaw / 2.0f);

    q->w = cos_x2 * cos_y2 * cos_z2 + sin_x2 * sin_y2 * sin_z2;
    q->x = sin_x2 * cos_y2 * cos_z2 - cos_x2 * sin_y2 * sin_z2;
    q->y = cos_x2 * sin_y2 * cos_z2 + sin_x2 * cos_y2 * sin_z2;
    q->z = cos_x2 * cos_y2 * sin_z2 - sin_x2 * sin_y2 * cos_z2;

    /* Normalize to ensure unit quaternion */
    quat_normalize(q);
}
