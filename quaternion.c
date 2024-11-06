#include <math.h>
#include "quaternion.h"

void scale_quat(quat_t *q, float factor, quat_t *out) {
    out->w = q->w * factor;
    out->x = q->x * factor;
    out->y = q->y * factor;
    out->z = q->z * factor;
}

void normalise_quat(quat_t *q, quat_t *out) {
    float q_len = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

    out->w = q->w / q_len;
    out->x = q->x / q_len;
    out->y = q->y / q_len;
    out->z = q->z / q_len;
}

void add_quats(quat_t *q1, quat_t *q2, quat_t *out) {
    out->w = q1->w + q2->w;
    out->x = q1->x + q2->x;
    out->y = q1->y + q2->y;
    out->z = q1->z + q2->z;
}

void multiply_quats(quat_t *q1, quat_t *q2, quat_t *out) {
    out->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    out->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    out->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    out->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void quat_to_euler(quat_t *q, float out[3]) {
    /* float yaw = atan2(2.0 * (q->y * q->z + q->w * q->x), q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z); */
    /* float pitch = asin(-2.0 * (q->x * q->z - q->w * q->y)); */
    /* float roll = atan2(2.0 * (q->x * q->y + q->w * q->z), q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z); */
    /* out[0] = roll; */
    /* out[1] = pitch; */
    /* out[2] = yaw; */
	// roll (x-axis rotation)
    double sinr_cosp = 2 * (q->w * q->x + q->y * q->z);
    double cosr_cosp = 1 - 2 * (q->x * q->x + q->y * q->y);
    out[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q->w * q->y - q->x * q->z));
    double cosp = sqrt(1 - 2 * (q->w * q->y - q->x * q->z));
    out[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q->w * q->z + q->x * q->y);
    double cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
    out[2] = atan2(siny_cosp, cosy_cosp);
}

