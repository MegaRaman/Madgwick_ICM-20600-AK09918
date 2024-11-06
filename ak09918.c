#include "ak09918.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#include <string.h>


static const char *TAG = "AK09918 driver";

esp_err_t AK_burst_read(struct AK09918 *dev, uint8_t reg, uint8_t *val, uint8_t size) {
	return i2c_master_transmit_receive(dev->dev_handle, &reg, 1, val, size, -1);
}

esp_err_t AK_read_reg(struct AK09918 *dev, uint8_t reg, uint8_t *out) {
	return AK_burst_read(dev, reg, out, 1);
}

esp_err_t AK_write_reg(struct AK09918 *dev, uint8_t reg, uint8_t val) {
	uint8_t tmp[2] = { reg, val };
	return i2c_master_transmit(dev->dev_handle, tmp, 2, -1);
}

esp_err_t AK_check_ID(struct AK09918 *dev) {
	esp_err_t err;
	uint8_t id1, id2;
	err = AK_read_reg(dev, AK_WIA1, &id1);
	err |= AK_read_reg(dev, AK_WIA2, &id2);
	if (err == ESP_OK) {
		if (id1 != AK_WIA1_VAL || id2 != AK_WIA2_VAL) {
			ESP_LOGE(TAG, "Got wrong WHO_AM_I value\n");
			return ESP_FAIL;
		}
	}
	return err;
}

esp_err_t AK_init(struct AK09918 *dev, i2c_master_dev_handle_t *dev_handle) {
	dev->dev_handle = *dev_handle;
	esp_err_t err = AK_check_ID(dev);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Initialization failed\n");
		return err;
	}
	/* 10 Hz measurement frequency */
	err = AK_set_power_mode(dev, AK_CONT_MSR_1);

	memset(dev->compass_offset, 0, 3 * sizeof(float));

#ifdef MOTIONCAL_CALIB
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j)
				dev->matrix_scale[i][j] = 1.0f;
			else
				dev->matrix_scale[i][j] = 0.0f;
		}
	}
#else
	memset(dev->compass_scale, 1, 3 * sizeof(float));
#endif
	return err;

}

esp_err_t AK_self_test(struct AK09918* dev) {
	while (AK_read(dev) == ESP_ERR_NOT_FINISHED) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	int16_t test[3];
	AK_get_raw_compass(dev, test);
	if (test[0] < -200 || test[0] > 200)
		return ESP_FAIL;
	if (test[1] < -200 || test[1] > 200)
		return ESP_FAIL;
	if (test[2] < -1000 || test[2] > -150)
		return ESP_FAIL;
	return ESP_OK;
}

/* Transition to POWER_DOWN mode for at least 100 us required between any mode
 * change */
esp_err_t AK_set_power_mode(struct AK09918 *dev, compass_mode_t mode) {
	esp_err_t err;

	if (mode != AK_POWER_DOWN && dev->power_mode != AK_POWER_DOWN) {
		err = AK_write_reg(dev, AK_CNTL2, AK_POWER_DOWN);
		if (err != ESP_OK)
			return err;
		/* 1000 us delay */
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	err = AK_write_reg(dev, AK_CNTL2, mode);
	dev->power_mode = mode;
	if (mode == AK_SELF_TEST) {
		if (AK_self_test(dev) == ESP_FAIL) {
			ESP_LOGE(TAG, "Self-test failed\n");
			return ESP_FAIL;
		}
	}
	return err;
}


esp_err_t AK_read(struct AK09918 *dev) {
	uint8_t st1;
	AK_read_reg(dev, AK_ST1, &st1);
	if ((st1 & AK_DRDY_MASK) == 0) {
		/* ESP_LOGI(TAG, "Data isn't ready for read\n"); */
		return ESP_ERR_NOT_FINISHED;
	}

	uint8_t tmp[8];
	esp_err_t err = AK_burst_read(dev, AK_HXL, tmp, 8);
	if (err == ESP_OK) {
		if ((tmp[7] & AK_HOFL_MASK) == AK_HOFL_MASK) {
			ESP_LOGE(TAG, "Sensor overflow detected, |x| + |y| + |z| >= 4912 uT\n");
			return ESP_FAIL;
		}
	}
	dev->raw_compass[0] = (tmp[1] << 8) | tmp[0];
	dev->raw_compass[1] = (tmp[3] << 8) | tmp[2];
	dev->raw_compass[2] = (tmp[5] << 8) | tmp[4];
	/* If device was set in one of those power modes then the sensor should've
	 * automatically transfer to POWER_DOWN mode */
	if (dev->power_mode == AK_SINGLE_MSR || dev->power_mode == AK_SELF_TEST)
		dev->power_mode = AK_POWER_DOWN;

	return err;
}

void AK_get_raw_compass(struct AK09918 *dev, int16_t *out) {
	memcpy(out, dev->raw_compass, 3 * sizeof(int16_t));
}

void AK_get_compass(struct AK09918 *dev, float *out) {
#ifdef MOTIONCAL_CALIB
	float hi_cal[3];
	for (int i = 0; i < 3; i++)
		hi_cal[i] = dev->raw_compass[i] * AK_COMPASS_SCALE - dev->compass_offset[i];

	for (int i = 0; i < 3; i++) {
		dev->compass[i] = (dev->matrix_scale[i][0] * hi_cal[0]) +
						 (dev->matrix_scale[i][1] * hi_cal[1]) +
						 (dev->matrix_scale[i][2] * hi_cal[2]);

	}
#else
	for (int i = 0; i < 3; i++)
		dev->compass[i] = (dev->raw_compass[i] * AK_COMPASS_SCALE - dev->compass_offset[i]) * dev->compass_scale[i];
#endif
	memcpy(out, dev->compass, 3 * sizeof(float));
}

// credits: https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
void AK_calibrate_compass(struct AK09918* dev, float *offset, float *scale) {
#ifndef MOTIONCAL_CALIB
	ESP_LOGI(TAG, "Move your magnetometer in 8-figure across all axes");
	int16_t mag_max[3] = { SHRT_MIN, SHRT_MIN, SHRT_MIN};
	int16_t mag_min[3] = { SHRT_MAX, SHRT_MAX, SHRT_MAX};
	int16_t msr[3];

	TickType_t calib_start_time = xTaskGetTickCount();
	while (xTaskGetTickCount() - calib_start_time < pdMS_TO_TICKS(AK_CALIBRATION_TIME_MS)) {
		if (AK_read(dev) != ESP_OK) {
			continue;
		}
		AK_get_raw_compass(dev, msr);
		for (int i = 0; i < 3; i++) {
			mag_max[i] = (msr[i] > mag_max[i]) ? msr[i] : mag_max[i];
			mag_min[i] = (msr[i] < mag_min[i]) ? msr[i] : mag_min[i];
		}
	}
	for (int i = 0; i < 3; i++)
		dev->compass_offset[i] = (mag_max[i] + mag_min[i]) / 2.0 * AK_COMPASS_SCALE;

	scale[0] = (mag_max[0] - mag_min[0]) / 2.0;  // get average x axis max chord length
	scale[1] = (mag_max[1] - mag_min[1]) / 2.0;  // get average y axis max chord length
	scale[2] = (mag_max[2] - mag_min[2]) / 2.0;  // get average z axis max chord length

	float avg_rad = (scale[0] + scale[1] + scale[2]) / 3.0;
	scale[0] = avg_rad / scale[0];
	scale[1] = avg_rad / scale[1];
	scale[2] = avg_rad / scale[2];

	memcpy(dev->compass_scale, scale, 3 * sizeof(float));

	ESP_LOGI(TAG, "Calibration is finished\n");
#else
	ESP_LOGE(TAG, "Function AK_calibrate_compass shouldn't be used with MOTIONCAL_CALIB defined");
#endif
}

void AK_set_calibration_params(struct AK09918 *dev, float *offset, float *scale) {
#ifndef MOTIONCAL_CALIB
	memcpy(dev->compass_offset, offset, 3 * sizeof(float));
	memcpy(dev->compass_scale, scale, 3 * sizeof(float));
#else
	ESP_LOGE(TAG, "Function AK_set_calibration_params shouldn't be used with MOTIONCAL_CALIB defined");
#endif
}

void AK_set_calibration_params_matrix(struct AK09918 *dev, const float *offset, const float scale[3][3]) {
#ifdef MOTIONCAL_CALIB
	memcpy(dev->compass_offset, offset, 3 * sizeof(float));
	memcpy(dev->matrix_scale, scale, 3 * 3 * sizeof(float));
#else
	ESP_LOGE(TAG, "Function AK_set_calibration_params_matrix should be used with MOTIONCAL_CALIB defined");
#endif
}

