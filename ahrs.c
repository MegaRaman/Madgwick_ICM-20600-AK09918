#include <math.h>
#include "ahrs.h"
#include "quaternion.h"

#define RAD_TO_DEG          (57.2958f)
#define DEG_TO_RAD          (1.0f / RAD_TO_DEG)

void MadgwickQuaternionUpdate(struct AHRS_filter *filter, float ax, float ay,
		float az, float gx, float gy, float gz, float mx, float my, float mz) {
	float q1 = filter->q.w, q2 = filter->q.x, q3 = filter->q.y, q4 = filter->q.z;   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;
	//            float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	mx /= 1000;
	my /= 1000;
	mz /= 1000;
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	/*            // Compute estimated gyroscope biases
				  gerrx = _2q1 * s2 - _2q2 * s1 - _2q3 * s4 + _2q4 * s3;
				  gerry = _2q1 * s3 + _2q2 * s4 - _2q3 * s1 - _2q4 * s2;
				  gerrz = _2q1 * s4 - _2q2 * s3 + _2q3 * s2 - _2q4 * s1;

	// Compute and remove gyroscope biases
	gbiasx += gerrx * deltat * zeta;
	gbiasy += gerry * deltat * zeta;
	gbiasz += gerrz * deltat * zeta;
	gx -= gbiasx;
	gy -= gbiasy;
	gz -= gbiasz;
	*/
	// Compute rate of change of quaternion
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - filter->beta * s1;
	qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - filter->beta * s2;
	qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - filter->beta * s3;
	qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - filter->beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * filter->sampling_period_ms / 1000.0f;
	q2 += qDot2 * filter->sampling_period_ms / 1000.0f;
	q3 += qDot3 * filter->sampling_period_ms / 1000.0f;
	q4 += qDot4 * filter->sampling_period_ms / 1000.0f;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f / norm;
	filter->q.w = q1 * norm;
	filter->q.x = q2 * norm;
	filter->q.y = q3 * norm;
	filter->q.z = q4 * norm;
}

void AHRS_estimate(struct AHRS_filter *filter, quat_t *out) {
	MadgwickQuaternionUpdate(filter, filter->acc_gyro->acc[0],
			filter->acc_gyro->acc[1], filter->acc_gyro->acc[2],
			filter->acc_gyro->gyro[0], filter->acc_gyro->gyro[1],
			filter->acc_gyro->gyro[2], filter->compass->compass[0],
			filter->compass->compass[1], filter->compass->compass[2]);
	*out = filter->q;
}

void AHRS_init(struct AHRS_filter *filter, struct ICM20600 *acc_gyro,
		struct AK09918 *compass, uint32_t sampling_period_ms) {
	filter->acc_gyro = acc_gyro;
	filter->compass = compass;
	filter->sampling_period_ms = sampling_period_ms;
	filter->beta = 0.866f * 1.04;
	filter->q.w = 1;
	filter->q.x = 0;
	filter->q.y = 0;
	filter->q.z = 0;
}


