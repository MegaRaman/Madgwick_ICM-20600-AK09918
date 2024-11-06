# ESP IDF Attitude Estimation with Madgwick Filter using Grove IMU 9 DOF(ICM-20600 & AK09918)

## Credits
The original paper on Madgwick filter can be found 
[here](https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf), implementation was taken
from [kriswiner repo](https://github.com/kriswiner/MPU9250/blob/master/quaternionFilters.ino), also method for magnetometer
calibration was very accessibly explained in 
[this repo](https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration) of the same author.

## Overview
This repo contains two essential parts: driver for IMU sensors and AHRS system that uses Madgwick filter with appropriate beta. 
Note that task of compass calibration is very essential for attitude estimation so two possibilities are implemented: one 
requires additional software: MotionCal or similar tool for generating hard-iron and the other is a simpler method that is
implemented right inside the driver. I wasn't successful in using simpler kriswiner's calibration, so I had to use MotionCal, 
data format for it can be found commented in `orientation_imu.c`. 

## Interfaces

### Magnetometer calibration
- To choose the simpler calibration method comment the following define: `#define MOTIONCAL_CALIB 1` in `ak09918.h`
- To choose software that gives soft-iron calibration 3x3 matrix uncomment define if it was commented
- To use simple compass calibration use `AK_calibrate_compass()`, in this case to set calibration time change 
`AK_CALIBRATION_TIME_MS` macro defined in compass header file, to use parameters found with MotionCal follow next paragraph
- If calibration parameters was already determined use `AK_set_calibration_params_matrix()` for MotionCal calibration 
and `AK_set_calibration_params()` for simple kriswiner's calibration

### AK09918 magnetometer driver
- Use `struct AK09918` to create new device that will contain all relevant information
- To init device use `esp_err_t AK_init(struct AK09918 *dev, i2c_master_dev_handle_t *dev_handle)`, register dev_handle as showed in orientation_imu.c.
That will initialize device in `AK_CONT_MSR_1` mode, which means 10 Hz frequency. You can choose another using 
`esp_err_t AK_set_power_mode(struct AK09918 *dev, compass_mode_t mode)`
- To initialize new data read from magnetometer use `esp_err_t AK_read(struct AK09918 *dev)`
- To read received raw data use `void AK_get_raw_compass(struct AK09918 *dev, int16_t *out)`, where `out` is of type `int16_t[3]`
- To read received data in uT use `void AK_get_compass(struct AK09918 *dev, float *out)`, where `out` is of type `float[3]`, 
that also will apply calibration if it was set

### ICM-20600 accelerometer & gyroscope
- Use `struct ICM20600` to create new device that will contain all relevant information
- To init device use `esp_err_t ICM_init(struct ICM20600 *dev, i2c_master_dev_handle_t *dev_handle)`, register dev_handle as showed in orientation_imu.c.
That will initialize: gyro lowpass filter to 176 Hz 3-dB BW, rate to 1 kHz; temperature sensor lowpass to 188 Hz 3-dB BW; accelerometer lowpass to 99 Hz cutoff;
By default FIFO and FSYNC are disabled, as well as any interrupts, no interfaces to change it.
Device mode will be set to `ICM_LN_MODE`, which can be changed through `esp_err_t ICM_set_power_mode(struct ICM20600 *dev, ICM_mode_t power_mode)`
- Gyroscope full scale - +-250 dps, accelerometer - +-2g
- To initialize new data read from device use `esp_err_t ICM_read(struct ICM20600 *dev)`
- To read received raw data use `void ICM_get_raw_acc/gyro/temp(struct ICM20600 *dev, int16_t *out)`
- To read received data in G/dps/degC use `void ICM_get_acc/gyro/temp(struct ICM20600 *dev, float *out)`

### Madgwick orientation estimation
- Use `struct AHRS_filter` to create an instance of filter
- Use `void AHRS_init(struct AHRS_filter *filter, struct ICM20600 *acc_gyro, struct AK09918 *compass, uint32_t sampling_period_ms)` to initialize it
- Use `void AHRS_estimate(struct AHRS_filter *filter, quat_t *out)` to estimate position, you can translate output quaternion to Euler angles using 
`void quat_to_euler(quat_t *q, float out[3])`

## Usage
You can use it as a standalone project or as an ESP IDF component by cloning this project to /components directory. Current state of the repo just prints the
roll/pitch/yaw angles.

## TODO
- Instead of printing angles orientation of sensor could be visualized
- MotionCal functionality could also be implemented inside the project

