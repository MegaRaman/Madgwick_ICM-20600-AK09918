#ifndef __AK09918_H__
#define __AK09918_H__

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTIONCAL_CALIB 1
#define AK_CALIBRATION_TIME_MS (25000)


#define AK_I2C_ADDR 0x0c
#define AK_COMPASS_SCALE (0.15) // sensitivity of device in uT/LSB


enum AK_regs {
	AK_WIA1 = 0x00,
	AK_WIA2 = 0x01,
	AK_ST1 = 0x10,
	AK_HXL = 0x11,
	AK_HXH = 0x12,
	AK_HYL = 0x13,
	AK_HYH = 0x14,
	AK_HZL = 0x15,
	AK_HZH = 0x16,
	AK_ST2 = 0x18,
	AK_CNTL2 = 0x31,
	AK_CNTL3 = 0x32,
};

enum AK_configs {
	AK_WIA1_VAL = 0x48,
	AK_WIA2_VAL = 0x0C,
	AK_DRDY_MASK = 0x01,
	AK_HOFL_MASK = 0x08,
};

typedef enum {
	AK_POWER_DOWN = 0x00,
	AK_SINGLE_MSR = 0x01,
	AK_CONT_MSR_1 = 0x02,
	AK_CONT_MSR_2 = 0x04,
	AK_CONT_MSR_3 = 0x08,
	AK_CONT_MSR_4 = 0x09,
	AK_SELF_TEST = 0x10,
} compass_mode_t;

struct AK09918 {
	i2c_master_dev_handle_t dev_handle;
	compass_mode_t power_mode;

	int16_t raw_compass[3];
	float compass[3];
	float compass_offset[3];
#ifdef MOTIONCAL_CALIB
	float matrix_scale[3][3];
#else
	float compass_scale[3];
#endif
};

esp_err_t AK_init(struct AK09918 *dev, i2c_master_dev_handle_t *dev_handle);
esp_err_t AK_set_power_mode(struct AK09918 *dev, compass_mode_t mode);

esp_err_t AK_read(struct AK09918 *dev);
void AK_get_raw_compass(struct AK09918 *dev, int16_t *out);
void AK_get_compass(struct AK09918 *dev, float *out);

void AK_calibrate_compass(struct AK09918* dev, float *offset, float *scale);
void AK_set_calibration_params(struct AK09918 *dev, float *offset, float *scale);
void AK_set_calibration_params_matrix(struct AK09918 *dev, const float *offset, const float scale[3][3]);

#endif // __AK09918_H__

