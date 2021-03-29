#include "drv_mpu9250.h"
#include "drv_mpu_regs.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "drv_i2c.h"
#include "drv_mpu9250_const.h"
#include "board.h"
#include "util.h"

/* MPU9250 I2C device */
static i2c_device_t mpu9250;

/* Internal accelerometer variables */
static float acc_scale_val = 0.0;

/* Accelerometer scaling correction */
static mpu9250_acc_data_t acc_scale_err_pos = {.x = ACC_SCALE_POS_1,
                                               .y = ACC_SCALE_POS_2,
                                               .z = ACC_SCALE_POS_3};
static mpu9250_acc_data_t acc_scale_err_neg = {.x = ACC_SCALE_NEG_1,
                                               .y = ACC_SCALE_NEG_2,
                                               .z = ACC_SCALE_NEG_3};

/* Internal gyroscope variables */
static float gyr_scale_val = 0.0;
static mpu9250_gyr_data_t gyr_bias_err;

esp_err_t drv_mpu9250_init() {

	/* Set I2C parameter */
	mpu9250.port = 0;
	mpu9250.scl = 32;
	mpu9250.sda = 33;
	mpu9250.addr = MPU9250_ADDR;
	mpu9250.speed = 400000;
	mpu9250.timeout = 1000;

	esp_err_t ret_mpu9250 = drv_i2c_init(mpu9250);

	if (drv_mpu9250_ping()) {
		ESP_LOGE(__FILE__,"MPU9250 not found");
		return ESP_FAIL;
	}

	/* Configure MPU9250 I2C connection */
	ret_mpu9250 |= drv_mpu9250_reset();
	delay_ms(500);
	ret_mpu9250 |= drv_mpu9250_config_clk();
	delay_ms(100);
	ret_mpu9250 |= drv_mpu9250_enable_mag_access();
	delay_ms(10);
	ret_mpu9250 |= drv_mpu9250_set_acc_filter();
	delay_ms(10);
	ret_mpu9250 |= drv_mpu9250_set_acc_scale(AS_4);  // 4g
	delay_ms(10);
	ret_mpu9250 |= drv_mpu9250_set_gyr_filter();
	delay_ms(10);
	ret_mpu9250 |= drv_mpu9250_set_gyr_scale(GS_500);  // 500 degree per second
	delay_ms(10);
	ret_mpu9250 |= drv_mpu9250_enable_sensors();
	delay_ms(100);
	ESP_LOGI(__FILE__,"Calibrating MPU9250 ...");
	ret_mpu9250 |= drv_mpu9250_calibrate(&gyr_bias_err);

	// printf("gyr_bias_err x %10.5f, y %10.5f, z %10.5f \n",
	//        gyr_bias_err.x, gyr_bias_err.y, gyr_bias_err.z);

	return ret_mpu9250;
}

esp_err_t drv_mpu9250_reset() {

	drv_i2c_write_bytes(mpu9250, PWR_MGMT_1, 1, MPU9250_BMASK_RESET);

	return ESP_OK;
}

esp_err_t drv_mpu9250_enable_sensors() {

	drv_i2c_write_bytes(mpu9250, PWR_MGMT_2, 1, MPU9250_BMASK_EN_ALL);

	return ESP_OK;
}

esp_err_t drv_mpu9250_disable_sensors() {

	drv_i2c_write_bytes(mpu9250, PWR_MGMT_2, 1, MPU9250_BMASK_DIS_ALL);

	return ESP_OK;
}

esp_err_t drv_mpu9250_enable_mag_access() {

	uint8_t data = 0x00;
	data &= ~MPU9250_BMASK_I2CMST_DIS;
	drv_i2c_write_bytes(mpu9250, USER_CTRL, 1, data);

	drv_i2c_write_bytes(mpu9250, INT_PIN_CFG, 1, MPU9250_BMASK_BYPASS_EN);

	return ESP_OK;
}

esp_err_t drv_mpu9250_ping() {

	uint8_t resp = 0x00;
	drv_i2c_read_bytes(mpu9250, WHO_AM_I, 1, &resp);

	if (resp != MPU9250_WHO_AM_I_RESP) {
		ESP_LOGE(__FILE__, "MPU9250 not found, ping unsuccessful, WHO_AM_I_RESP does not match");
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t drv_mpu9250_config_clk() {

	// Clear sleep mode
	drv_i2c_write_bytes(mpu9250, PWR_MGMT_1, 1, MPU9250_BMASK_CLEAR_PWR1);
	delay_ms(100);

	// Select clock source -> best
	drv_i2c_write_bytes(mpu9250, PWR_MGMT_1, 1, MPU9250_BMASK_SELCLK);
	delay_ms(100);

	// Select sample rate -> divider = 4
	drv_i2c_write_bytes(mpu9250, SMPLRT_DIV, 1, MPU9250_BMASK_SMPLRT_DIV);
	delay_ms(100);

	return ESP_OK;
}

esp_err_t drv_mpu9250_set_acc_filter() {

	uint8_t resp = 0x00;
	drv_i2c_read_bytes(mpu9250, ACCEL_CONFIG_2, 1, &resp);

	resp &= ~(0x0F);
	resp |= MPU9250_BMASK_ACC_LPFCFG;

	drv_i2c_write_bytes(mpu9250, ACCEL_CONFIG_2, 1, resp);

	return ESP_OK;
}

esp_err_t drv_mpu9250_set_acc_scale(mpu9250_acc_scale_t scale) {

	drv_i2c_write_bytes(mpu9250, ACCEL_CONFIG, 1, scale << 3);

	switch(scale) {
		case AS_2:
			acc_scale_val = 2.0/32768.0;
			break;
		case AS_4:
			acc_scale_val = 4.0/32768.0;
			break;
		case AS_8:
			acc_scale_val = 8.0/32768.0;
			break;
		case AS_16:
			acc_scale_val = 16.0/32768.0;
			break;
	}

	return ESP_OK;
}

esp_err_t drv_mpu9250_set_gyr_filter() {

	drv_i2c_write_bytes(mpu9250, CONFIG, 1, MPU9250_BMASK_GYR_LPFCFG);

	uint8_t resp = 0x00;
	drv_i2c_read_bytes(mpu9250, GYRO_CONFIG, 1, &resp);

	resp &= ~(MPU9250_BMASK_GYR_FCHOICEB);
	drv_i2c_write_bytes(mpu9250, GYRO_CONFIG, 1, resp);

	return ESP_OK;
}

esp_err_t drv_mpu9250_set_gyr_scale(mpu9250_gyr_scale_t scale) {

	uint8_t data;
	drv_i2c_read_bytes(mpu9250, GYRO_CONFIG, 1, &data);

	data &= ~(0x18);
	data |= (scale << 3);

	drv_i2c_write_bytes(mpu9250, GYRO_CONFIG, 1, data);

	switch(scale) {
		case GS_250:
			gyr_scale_val = 250.0/32768.0;
			break;
		case GS_500:
			gyr_scale_val = 500.0/32768.0;
			break;
		case GS_1000:
			gyr_scale_val = 1000.0/32768.0;
			break;
		case GS_2000:
			gyr_scale_val = 2000.0/32768.0;
			break;
	}

	return ESP_OK;
}

esp_err_t drv_mpu9250_read_acc_native(mpu9250_acc_data_t *acc) {

	uint8_t raw[6];
	drv_i2c_read_bytes(mpu9250, ACCEL_XOUT_H, 6, raw);

	/* Scale accelerometer values according to mpu settings */
	acc->x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]) * acc_scale_val;
	acc->y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]) * acc_scale_val;
	acc->z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]) * acc_scale_val;

	/* Scale error correction */
	acc->x *= acc->x > 0.0 ? acc_scale_err_pos.x : acc_scale_err_neg.x;
	acc->y *= acc->y > 0.0 ? acc_scale_err_pos.y : acc_scale_err_neg.y;
	acc->z *= acc->z > 0.0 ? acc_scale_err_pos.z : acc_scale_err_neg.z;

	return ESP_OK;
}

/* Returns the orientation and scale corrected accelerometer data
   on all three axis in g (multiple of 9.80665m/s^2). */
esp_err_t drv_mpu9250_read_acc(mpu9250_acc_data_t *acc) {

	mpu9250_acc_data_t acc_tmp;
	drv_mpu9250_read_acc_native(&acc_tmp);

	#if defined(NATIVE_ORIENT)
		acc->x = acc_tmp.x;
		acc->y = acc_tmp.y;
		acc->z = acc_tmp.z;
	#elif defined(DEFAULT_ORIENT)
		acc->x = acc_tmp.y;
		acc->y = acc_tmp.x;
		acc->z = -acc_tmp.z;
	#elif defined(HYPE_ORIENT)
		acc->x = acc_tmp.y;
		acc->y = -acc_tmp.z;
		acc->z = -acc_tmp.x;
	#else
		#error "Unkown board orientation"
	#endif

	return ESP_OK;
}

esp_err_t drv_mpu9250_read_gyr_native(mpu9250_gyr_data_t *gyr) {

	uint8_t raw[6];
	drv_i2c_read_bytes(mpu9250, GYRO_XOUT_H, 6, raw);

	gyr->x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]) * gyr_scale_val;
	gyr->y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]) * gyr_scale_val;
	gyr->z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]) * gyr_scale_val;

	/* Convert from degree to rad */
	gyr->x *= M_PI/360.0;
	gyr->y *= M_PI/360.0;
	gyr->z *= M_PI/360.0;

	/* Bias error correction */
	gyr->x -= gyr_bias_err.x;
	gyr->y -= gyr_bias_err.y;
	gyr->z -= gyr_bias_err.z;

	return ESP_OK;
}

/* Returns the orientation and bias corrected gyroscope data
   on all three axis in rad/s (rad per second). */
esp_err_t drv_mpu9250_read_gyr(mpu9250_gyr_data_t *gyr) {

	mpu9250_gyr_data_t gyr_tmp;
	drv_mpu9250_read_gyr_native(&gyr_tmp);

	#if defined(NATIVE_ORIENT)
		gyr->x = gyr_tmp.x;
		gyr->y = gyr_tmp.y;
		gyr->z = gyr_tmp.z;
	#elif defined(DEFAULT_ORIENT)
		gyr->x = gyr_tmp.y;
		gyr->y = gyr_tmp.x;
		gyr->z = -gyr_tmp.z;
	#elif defined(HYPE_ORIENT)
		gyr->x = gyr_tmp.y;
		gyr->y = -gyr_tmp.z;
		gyr->z = -gyr_tmp.x;
	#else
		#error "Unkown board orientation"
	#endif

	return ESP_OK;
}

esp_err_t drv_mpu9250_calibrate(mpu9250_gyr_data_t *gyr) {

	#define NUM_CAL_SAMPLES (500)
	float gyr_raw[3][NUM_CAL_SAMPLES];
	TickType_t last_wake_time = xTaskGetTickCount();

	for (int i=0; i<NUM_CAL_SAMPLES; i++) {

		mpu9250_gyr_data_t gyr_tmp;
		if (drv_mpu9250_read_gyr_native(&gyr_tmp) != ESP_OK) {
			return ESP_FAIL;
		}

		/* Store values */
		gyr_raw[0][i] = gyr_tmp.x;
		gyr_raw[1][i] = gyr_tmp.y;
		gyr_raw[2][i] = gyr_tmp.z;

		/* Print progress */
		if ((i+1) % 50 == 0) {
			ESP_LOGI(__FILE__, "%d %%", (int)((i+1) / 5));
		}

		delay_until_ms(&last_wake_time, 20);
	}

	ESP_LOGI(__FILE__, "Std x: %f", stde(gyr_raw[0], NUM_CAL_SAMPLES));
	ESP_LOGI(__FILE__, "Std y: %f", stde(gyr_raw[1], NUM_CAL_SAMPLES));
	ESP_LOGI(__FILE__, "Std z: %f", stde(gyr_raw[2], NUM_CAL_SAMPLES));

	#define STD_THR 0.01
	if (stde(gyr_raw[0], NUM_CAL_SAMPLES) > STD_THR
	 || stde(gyr_raw[1], NUM_CAL_SAMPLES) > STD_THR
	 || stde(gyr_raw[2], NUM_CAL_SAMPLES) > STD_THR) {
		ESP_LOGE(__FILE__, "Calibration deviation too high");
		return ESP_FAIL;
	}

	/* Store means in error constants */
	gyr->x = mean(gyr_raw[0], NUM_CAL_SAMPLES);
	gyr->y = mean(gyr_raw[1], NUM_CAL_SAMPLES);
	gyr->z = mean(gyr_raw[2], NUM_CAL_SAMPLES);

	return ESP_OK;
}

/* Returns the temperature in °c (degree celsius) */
float drv_mpu9250_read_temp() {
	uint8_t tmp[2] = {0};
	drv_i2c_read_bytes(mpu9250, TEMP_OUT_H, 2, tmp);
	return (tmp[0]<<8|tmp[1])/100.0;
}

void drv_mpu9250_test() {

	ESP_LOGI(__FILE__, "MPU Driver Test");

	while(1) {

		mpu9250_acc_data_t acc;
		drv_mpu9250_read_acc(&acc);
		printf("acc x %10.5f, y %10.5f, z %10.5f ", acc.x, acc.y, acc.z);

		mpu9250_gyr_data_t gyr;
		drv_mpu9250_read_gyr(&gyr);
		printf("gyr x %10.5f, y %10.5f, z %10.5f \n", gyr.x, gyr.y, gyr.z);

		delay_ms(10);
	}

}
