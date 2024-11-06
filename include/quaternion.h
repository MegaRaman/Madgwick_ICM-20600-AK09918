#ifndef __QUATERNION_H__
#define __QUATERNION_H__

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_t;

void scale_quat(quat_t *q, float factor, quat_t *out);
void normalise_quat(quat_t *q, quat_t *out);
void add_quats(quat_t *q1, quat_t *q2, quat_t *out);
void multiply_quats(quat_t *q1, quat_t *q2, quat_t *out);
void quat_to_euler(quat_t *q, float out[3]);

#endif // __QUATERNION_H__

