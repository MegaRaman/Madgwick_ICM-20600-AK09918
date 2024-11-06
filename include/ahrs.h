#ifndef __AHRS_H__
#define __AHRS_H__

#include "ak09918.h"
#include "icm20600.h"
#include "quaternion.h"

#define RAD_TO_DEG          (57.2958f)
#define DEG_TO_RAD          (1.0f / RAD_TO_DEG)

struct AHRS_filter {
	quat_t q;
	struct ICM20600 *acc_gyro;
	struct AK09918 *compass;
	uint32_t sampling_period_ms;
	float beta;
};

void AHRS_init(struct AHRS_filter *filter, struct ICM20600 *acc_gyro,
		struct AK09918 *compass, uint32_t sampling_period_ms);
void AHRS_estimate(struct AHRS_filter *filter, quat_t *out);

#endif // __AHRS_H__

