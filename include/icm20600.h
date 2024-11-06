#ifndef __ICM20600_H__
#define __ICM20600_H__

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ICM_I2C_ADDR 0x69
#define ICM_WHO_AM_I_VAL 0x11

#define ICM_GYRO_SCALE 131.0f	// gyro sensitivity - 131 LSB/dps
#define ICM_ACC_SCALE 16384.0f	// accelerometer sensitivity - 16384 LSB/g
#define ICM_TEMP_SCALE 326.8f	// temp sensitivity - 326.8 LSB/degC
#define ICM_TEMP_OFFSET 25

#define G_TO_MPS2	9.8

enum ICM_regs {
	ICM_SMPLRT_DIV = 0x19,
	ICM_CONFIG = 0x1A,
	ICM_GYRO_CONFIG = 0x1B,
	ICM_ACCEL_CONFIG = 0x1C,
	ICM_ACCEL_CONFIG2 = 0x1D,
	ICM_GYRO_LP_MODE_CFG = 0x1E,
	ICM_FIFO_EN = 0x23,
	ICM_INT_EN = 0x38,
	ICM_ACCEL_XOUT_H = 0x3B,
	ICM_ACCEL_XOUT_L = 0x3C,
	ICM_ACCEL_YOUT_H = 0x3D,
	ICM_ACCEL_YOUT_L = 0x3E,
	ICM_ACCEL_ZOUT_H = 0x3F,
	ICM_ACCEL_ZOUT_L = 0x40,
	ICM_TEMP_OUT_H = 0x41,
	ICM_TEMP_OUT_L = 0x42,
	ICM_GYRO_XOUT_H = 0x43,
	ICM_GYRO_XOUT_L = 0x44,
	ICM_GYRO_YOUT_H = 0x45,
	ICM_GYRO_YOUT_L = 0x46,
	ICM_GYRO_ZOUT_H = 0x47,
	ICM_GYRO_ZOUT_L = 0x48,
	ICM_USER_CTRL = 0x6A,
	ICM_PWR_MGMT_1 = 0x6B,
	ICM_PWR_MGMT_2 = 0x6C,
	ICM_WHO_AM_I = 0x75,
};

enum ICM_configs {
	ICM_CONFIG_VAL = 0x01, // no FSYNC function, gyroscope cutoff - 176 Hz
	ICM_ACC_CONFIG_VAL = 0x02,
	ICM_ACC_DISABLE = 0x38,
	ICM_GYRO_DISABLE = 0x07,
	ICM_GYRO_ACC_ENABLE = 0x00,
	ICM_SLEEP_MODE_VAL = 0x40,
	ICM_STANDBY_MODE_VAL = 0x10, // 00010000
	ICM_ACC_LP_MODE = 0x20, // 00100000
	ICM_GYRO_DISABLE_LP = 0x00,
	ICM_GYRO_ENABLE_LP = 0x80, // 10000000
};

typedef enum {
	SLEEP_MODE,
	STANDBY_MODE,
	ACC_LP_MODE,
	ACC_LN_MODE,
	GYRO_LP_MODE,
	GYRO_LN_MODE,
	ICM_LN_MODE,
	ICM_LP_MODE,
} ICM_mode_t;


struct ICM20600 {
	i2c_master_dev_handle_t dev_handle;
	ICM_mode_t power_mode;

	int16_t raw_temp;
	float temp;
	int16_t raw_acc[3];
	float acc[3];
	int16_t raw_gyro[3];
	float gyro[3];
};

esp_err_t ICM_init(struct ICM20600 *dev, i2c_master_dev_handle_t *dev_handle);
esp_err_t ICM_set_smplrt_div(struct ICM20600 *dev, uint8_t div);
esp_err_t ICM_set_power_mode(struct ICM20600 *dev, ICM_mode_t power_mode);
esp_err_t ICM_read(struct ICM20600 *dev);

void ICM_get_raw_acc(struct ICM20600 *dev, int16_t *out);
void ICM_get_raw_temp(struct ICM20600 *dev, int16_t *temp);
void ICM_get_raw_gyro(struct ICM20600 *dev, int16_t *out);

void ICM_get_acc(struct ICM20600 *dev, float *out);
void ICM_get_temp(struct ICM20600 *dev, float *temp);
void ICM_get_gyro(struct ICM20600 *dev, float *out);

#endif // __IMU_H__

