#include "ahrs.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_log_internal.h"

/* #include <stdio.h> */

#include "ak09918.h"
#include "icm20600.h"
#include "quaternion.h"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define IMU_SAMPLE_PERIOD_MS		100

static const char* TAG = "Main Application";

static const float hard_iron[] = { -22.92, -2.78, 22.02 };
static const float soft_iron[3][3] = {
	{ 1.008, -0.012, -0.025 },
	{-0.012, 0.964, 0.06},
	{-0.025, 0.06, 1.033}
};

void app_main(void)
{
	esp_err_t err;
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = -1,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = ICM_I2C_ADDR,
		.scl_speed_hz = 100000,
	};

	i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

	struct ICM20600 acc_gyro;
	err = ICM_init(&acc_gyro, &dev_handle);
	if (err != ESP_OK) {
		while (1) {
			ESP_LOGE(TAG, "Error initializing ICM20600\n");
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}
	i2c_device_config_t dev_cfg_compass = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = AK_I2C_ADDR,
		.scl_speed_hz = 100000,
	};

	i2c_master_dev_handle_t dev_handle_compass;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_compass, &dev_handle_compass));
	struct AK09918 compass;
	err = AK_init(&compass, &dev_handle_compass);
	if (err != ESP_OK) {
		while (1) {
			ESP_LOGE(TAG, "Error initializing AK09918\n");
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}
	struct AHRS_filter filter;
	AHRS_init(&filter, &acc_gyro, &compass, IMU_SAMPLE_PERIOD_MS);

	/* float offset[3] = { -29.400000 ,-1.950000 ,-9.825000 }; */
	/* float scale[3] = { 0.917508, 0.789855, 1.552707 }; */
	/* AK_set_calibration_params(&compass, offset, scale); */

	/* float offset[3]; */
	/* float scale[3]; */
	/* AK_calibrate_compass(&compass, offset, scale); */
	/* ESP_LOGI(TAG, "Offset x: %9.6f y: %9.6f z: %9.6f\n", offset[0], offset[1], offset[2]); */
	/* ESP_LOGI(TAG, "Scale x: %9.6f y: %9.6f z: %9.6f\n", scale[0], scale[1], scale[2]); */
	/* vTaskDelay(pdMS_TO_TICKS(5000)); */

	AK_set_calibration_params_matrix(&compass, hard_iron, soft_iron);

	while (1) {
		err = ICM_read(&acc_gyro);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Error reading ICM20600\n");
		}
		err = AK_read(&compass);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Error reading AK09918\n");
		}
		float out[3];
		ICM_get_acc(&acc_gyro, out);
		/* ESP_LOGI(TAG, "Accelerometer x: %f, y: %f, z: %f\n", x, y, z); */
		ICM_get_gyro(&acc_gyro, out);
		/* ESP_LOGI(TAG, "Gyroscope x: %f, y: %f, z: %f\n", x, y, z); */
		AK_get_compass(&compass, out);
		/* ESP_LOGI(TAG, "Compass x: %f, y: %f, z: %f\n", x, y, z); */
		/* printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n", 0, 0, 0, 0, 0, 0, (int)x * 10, (int)y * 10, (int)z * 10); */

		quat_t pos;
		float pos_euler[3];
		AHRS_estimate(&filter, &pos);
		quat_to_euler(&pos, pos_euler);
		ESP_LOGI(TAG, "Roll: %9.5f Pitch: %9.5f Yaw: %9.5f\n",
				pos_euler[0] * RAD_TO_DEG,
				pos_euler[1] * RAD_TO_DEG,
				pos_euler[2] * RAD_TO_DEG);

		vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
	}
	return;
}

