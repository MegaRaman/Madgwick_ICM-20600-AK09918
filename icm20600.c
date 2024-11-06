#include "icm20600.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

const char *TAG = "ICM20600 driver";


esp_err_t ICM_burst_read(struct ICM20600 *dev, uint8_t reg, uint8_t *val, uint8_t size) {
	return i2c_master_transmit_receive(dev->dev_handle, &reg, 1, val, size, -1);
}

esp_err_t ICM_read_reg(struct ICM20600 *dev, uint8_t reg, uint8_t *val) {
	return ICM_burst_read(dev, reg, val, 1);
}

esp_err_t ICM_write_reg(struct ICM20600 *dev, uint8_t reg, uint8_t val) {
	uint8_t tmp[2] = { reg, val };
	return i2c_master_transmit(dev->dev_handle, tmp, 2, -1);
}

esp_err_t ICM_check_id(struct ICM20600 *dev) {
	uint8_t id;
	uint8_t reg = ICM_WHO_AM_I;
	esp_err_t err = ICM_read_reg(dev, reg, &id);

	if (err == ESP_OK) {
		if (id != ICM_WHO_AM_I_VAL) {
			ESP_LOGE(TAG, "Got wrong WHO_AM_I value\n");
			return ESP_FAIL;
		}
	}

	return err;
}

/* Gyroscope full scale - +-250 dps, accelerometer - +-2g */
esp_err_t ICM_init(struct ICM20600 *dev, i2c_master_dev_handle_t *dev_handle) {
	dev->dev_handle = *dev_handle;
	esp_err_t err = ICM_check_id(dev);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "ICM20600 initialization failed\n");
		return err;
	}
	/* New data in FIFO will overwrite old, FSYNC disabled, lowpass for gyro and
	 * temperature sensor is set that data rate is equal to 1kHz and
	 * cutoff is 176 Hz */
	err |= ICM_write_reg(dev, ICM_CONFIG, ICM_CONFIG_VAL);
	/* Set accelerometer DLPF to 99 Hz cutoff */
	err |= ICM_write_reg(dev, ICM_ACCEL_CONFIG2, ICM_ACC_CONFIG_VAL);
	err |= ICM_set_power_mode(dev, ICM_LN_MODE);

	return err;
}

esp_err_t ICM_set_power_mode(struct ICM20600 *dev, ICM_mode_t power_mode) {
	esp_err_t err;
	uint8_t pwr_mgmt_1, pwr_mgmt_2 = ICM_GYRO_ACC_ENABLE, gyro_lp_mode = ICM_GYRO_DISABLE_LP;
	err = ICM_read_reg(dev, ICM_PWR_MGMT_1, &pwr_mgmt_1);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set power mode\n");
		return err;
	}
	switch (power_mode) {
		case SLEEP_MODE:
			pwr_mgmt_1 |= ICM_SLEEP_MODE_VAL;
			break;
		case STANDBY_MODE:
			pwr_mgmt_1 &= ~ICM_SLEEP_MODE_VAL;
			pwr_mgmt_1 |= ICM_STANDBY_MODE_VAL;
			pwr_mgmt_2 = ICM_ACC_DISABLE;
			gyro_lp_mode = ICM_GYRO_DISABLE_LP;
			break;
		case ACC_LP_MODE:
			pwr_mgmt_1 &= ~ICM_SLEEP_MODE_VAL;
			pwr_mgmt_1 |= ICM_ACC_LP_MODE;
			pwr_mgmt_2 = ICM_GYRO_DISABLE;
			break;
		case ACC_LN_MODE:
			pwr_mgmt_1 &= ~ICM_SLEEP_MODE_VAL;
			pwr_mgmt_1 &= ~ICM_ACC_LP_MODE; // Clear CYCLE if it was set
			pwr_mgmt_2 = ICM_GYRO_DISABLE;
			break;
		case GYRO_LP_MODE:
			pwr_mgmt_1 &= ~ICM_SLEEP_MODE_VAL;
			pwr_mgmt_1 &= ~ICM_STANDBY_MODE_VAL; // Clear STANDBY if it was set
			pwr_mgmt_2 = ICM_ACC_DISABLE;
			gyro_lp_mode = ICM_GYRO_ENABLE_LP;
			break;
		case GYRO_LN_MODE:
			pwr_mgmt_1 &= ~ICM_SLEEP_MODE_VAL;
			pwr_mgmt_1 &= ~ICM_STANDBY_MODE_VAL; // Clear STANDBY if it was set
			pwr_mgmt_2 = ICM_ACC_DISABLE;
			gyro_lp_mode = ICM_GYRO_DISABLE_LP;
			break;
		case ICM_LP_MODE:
			pwr_mgmt_1 &= ~ICM_SLEEP_MODE_VAL;
			pwr_mgmt_1 &= ~ICM_STANDBY_MODE_VAL; // Clear STANDBY if it was set
			pwr_mgmt_1 |= ICM_ACC_LP_MODE;
			pwr_mgmt_2 = ICM_GYRO_ACC_ENABLE;
			gyro_lp_mode = ICM_GYRO_ENABLE_LP;
			break;
		case ICM_LN_MODE:
			pwr_mgmt_1 &= ~ICM_SLEEP_MODE_VAL;
			pwr_mgmt_1 &= ~ICM_STANDBY_MODE_VAL; // Clear STANDBY if it was set
			pwr_mgmt_1 &= ~ICM_ACC_LP_MODE; // Clear CYCLE if it was set
			pwr_mgmt_2 = ICM_GYRO_ACC_ENABLE;
			gyro_lp_mode = ICM_GYRO_DISABLE_LP;
			break;
		default:
			ESP_LOGE(TAG, "Invalid power mode argument\n");
			return ESP_FAIL;
	}
	err = ICM_write_reg(dev, ICM_PWR_MGMT_1, pwr_mgmt_1);
	err |= ICM_write_reg(dev, ICM_PWR_MGMT_2, pwr_mgmt_2);
	err |= ICM_write_reg(dev, ICM_GYRO_LP_MODE_CFG, gyro_lp_mode);
	dev->power_mode = power_mode;
	return err;
}

esp_err_t ICM_set_smplrt_div(struct ICM20600 *dev, uint8_t div) {
	return ICM_write_reg(dev, ICM_SMPLRT_DIV, div);
}

esp_err_t ICM_read(struct ICM20600 *dev) {
	esp_err_t err;
	uint8_t tmp[14];
	err = ICM_burst_read(dev, ICM_ACCEL_XOUT_H, tmp, 14);
	dev->raw_acc[0] = (tmp[0] << 8) | tmp[1];
	dev->raw_acc[1] = (tmp[2] << 8) | tmp[3];
	dev->raw_acc[2] = (tmp[4] << 8) | tmp[5];
	dev->raw_temp = (tmp[6] << 8) | tmp[7];
	dev->raw_gyro[0] =(tmp[8] << 8) | tmp[9];
	dev->raw_gyro[1] =(tmp[10] << 8) | tmp[11];
	dev->raw_gyro[2] =(tmp[12] << 8) | tmp[13];
	return err;
}

void ICM_get_raw_acc(struct ICM20600 *dev, int16_t *out) {
	memcpy(out, dev->raw_acc, 3 * sizeof(int16_t));
}

void ICM_get_raw_temp(struct ICM20600 *dev, int16_t *temp) {
	*temp = dev->raw_temp;
}

void ICM_get_raw_gyro(struct ICM20600 *dev, int16_t *out) {
	memcpy(out, dev->raw_gyro, 3 * sizeof(int16_t));
}

void ICM_get_acc(struct ICM20600 *dev, float *out) {
	for (int i = 0; i < 3; i++)
		dev->acc[i] = dev->raw_acc[i] / ICM_ACC_SCALE;
	memcpy(out, dev->acc, 3 * sizeof(float));
}

void ICM_get_temp(struct ICM20600 *dev, float *temp) {
	dev->temp = dev->raw_temp / ICM_TEMP_SCALE + ICM_TEMP_OFFSET;
	*temp = dev->temp;
}

void ICM_get_gyro(struct ICM20600 *dev, float *out) {
	for (int i = 0; i < 3; i++)
		dev->gyro[i] = dev->raw_gyro[i] / ICM_GYRO_SCALE;
	memcpy(out, dev->gyro, 3 * sizeof(float));
}


