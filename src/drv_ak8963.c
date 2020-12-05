#include "drv_ak8963.h"
#include "drv_ak_regs.h"
#include "drv_ak8963_const.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "drv_i2c.h"
#include "board.h"
#include "util.h"
#include "util_vecmath.h"

/* !!! If CALIBRATION activated, ensure orientation is set to NATIVE_ORIENT !!! */
#define CALIBRATION

/* AK8963 I2C device */
static i2c_device_t ak8963;

/* Internal gyroscope variables */
static ak8963_sens_coeff_t sens_coeff;

/* TODO Are determined once */
static ak8963_mag_data_t hard_iron_err = {.x = MAG_HI_1,
                                          .y = MAG_HI_2,
                                          .z = MAG_HI_3};
static float soft_iron_err[3][3] = {{MAG_SI_11, MAG_SI_12, MAG_SI_13},
                                    {MAG_SI_21, MAG_SI_22, MAG_SI_23},
                                    {MAG_SI_31, MAG_SI_32, MAG_SI_33}};

esp_err_t drv_ak8963_init() {

	ak8963.port = 0;
	ak8963.scl = 32;
	ak8963.sda = 33;
	ak8963.addr = AK8963_ADDR;
	ak8963.speed = 400000;
	ak8963.timeout = 1000;

	/* Already initalized by MPU9250 -> call mpu9250_init() before ak8963_init() */
	esp_err_t ret_ak8963 = ESP_OK; /* Not needed: = drv_i2c_init(ak8963); */

	if(drv_ak8963_ping()) {
		ESP_LOGE(__FILE__,"AK8963 not found");
		return ESP_FAIL;
	}

	ret_ak8963 |= drv_ak8963_read_sens_coeff();
	ret_ak8963 |= drv_ak8963_set_cntl1(CONTINUOUS2, BITOUT16);

	return ret_ak8963;
}

esp_err_t drv_ak8963_set_power_down() {
	drv_i2c_write_bytes(ak8963, CNTL1, 1, 0x00);
	return ESP_OK;
}

esp_err_t drv_ak8963_set_cntl1(ak8963_mode_t mode, ak8963_bit_t bit) {
	drv_i2c_write_bytes(ak8963, CNTL1, 1, (mode<<0) | (bit<<4));
	return ESP_OK;
}

esp_err_t drv_ak8963_ping() {
	uint8_t resp;
	drv_i2c_read_bytes(ak8963, WIA, 1, &resp);
	if (resp != AK8963_WHO_AM_I_RESP) {
		ESP_LOGE(__FILE__, "AK8963 not found, ping unsuccessful, WHO_AM_I_RESP does not match");
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t drv_ak8963_read_sens_coeff() {

	drv_ak8963_set_power_down();
	delay_ms(10);

	drv_i2c_write_bytes(ak8963, CNTL1, 1, FUSEROMACS);
	delay_ms(100);

	uint8_t data[3] = {0};
	drv_i2c_read_bytes(ak8963, ASAX, 3, data);
	sens_coeff.asa_x = data[0];
	sens_coeff.asa_y = data[1];
	sens_coeff.asa_z = data[2];
	sens_coeff.adj_x = (float)data[0]/256.0 + 0.5;
	sens_coeff.adj_y = (float)data[1]/256.0 + 0.5;
	sens_coeff.adj_z = (float)data[2]/256.0 + 0.5;
	// printf("adj_x = %f ", sens_coeff.adj_x);
	// printf("adj_y = %f ", sens_coeff.adj_y);
	// printf("adj_z = %f \n", sens_coeff.adj_z);

	drv_ak8963_set_power_down();
	delay_ms(10);

	return ESP_OK;
}

esp_err_t ak8963_reset() {
	drv_i2c_write_bytes(ak8963, CNTL2, 1, 0x01);
	return ESP_OK;
}

uint8_t drv_ak8963_data_rdy() {
	uint8_t data = 0x00;
	drv_i2c_read_bytes(ak8963, ST1, 1, &data);
	return (data & AK8963_BITMASK_DATA_RDY);
}

uint8_t drv_ak8963_status() {
	uint8_t data = 0x00;
	drv_i2c_read_bytes(ak8963, ST2, 1, &data);
	return (data & AK8963_BITMASK_HOFL);
}

esp_err_t drv_ak8963_read_mag_native(ak8963_mag_data_t *mag) {

	if (drv_ak8963_data_rdy()) {

		uint8_t data[6];
		drv_i2c_read_bytes(ak8963, HXL, 6, data);

		int16_t raw[3];
		raw[0] = (int16_t)((data[1] << 8) | data[0]);
		raw[1] = (int16_t)((data[3] << 8) | data[2]);
		raw[2] = -(int16_t)((data[5] << 8) | data[4]);

		#if !defined(CALIBRATION)
			float mag_tmp[3];
			mag_tmp[0] = (float)(raw[0] * sens_coeff.adj_x * AK8963_SENSITIVITY);
			mag_tmp[1] = (float)(raw[1] * sens_coeff.adj_y * AK8963_SENSITIVITY);
			mag_tmp[2] = (float)(raw[2] * sens_coeff.adj_z * AK8963_SENSITIVITY);
		#else
			mag->x = (float)(raw[0] * sens_coeff.adj_x * AK8963_SENSITIVITY);
			mag->y = (float)(raw[1] * sens_coeff.adj_y * AK8963_SENSITIVITY);
			mag->z = (float)(raw[2] * sens_coeff.adj_z * AK8963_SENSITIVITY);
		#endif

		#if !defined(CALIBRATION)
			/* Compensate for hard-iron errors */
			mag_tmp[0] -= hard_iron_err.x;
			mag_tmp[1] -= hard_iron_err.y;
			mag_tmp[2] -= hard_iron_err.z;

			/* Compensate for soft-iron errors */
			vecmath_matr_mul_3d(soft_iron_err, mag_tmp, (float*)mag);
		#endif

	} else {
		/* No new data available at this time */
		return ESP_ERR_TIMEOUT;
	}

	if (drv_ak8963_status() == 1) {
		ESP_LOGW(__FILE__, "Data overflow");
		return ESP_FAIL;
	}

	return ESP_OK;
}

/* Returns the orientation, soft- and hardiron corrected magetometer data
   on all three axis in uT (microtesla). */
esp_err_t drv_ak8963_read_mag(ak8963_mag_data_t *mag) {

	ak8963_mag_data_t mag_tmp;
	esp_err_t ret = drv_ak8963_read_mag_native(&mag_tmp);
	if (ret != ESP_OK) {
		return ret;
	}

	#if defined(NATIVE_ORIENT)
		mag->x = mag_tmp.y;
		mag->y = mag_tmp.x;
		mag->z = mag_tmp.z;
	#elif defined(DEFAULT_ORIENT)
		mag->x = mag_tmp.x;
		mag->y = mag_tmp.y;
		mag->z = -mag_tmp.z;
	#elif defined(HYPE_ORIENT)
		mag->x = mag_tmp.x;
		mag->y = mag_tmp.z;
		mag->z = -mag_tmp.y;
	#else
		#error "Unkown board orientation"
	#endif

	return ESP_OK;
}

void drv_ak8963_test() {

	ESP_LOGI(__FILE__, "AK Driver Test");

	while(1) {

		ak8963_mag_data_t mag;
		drv_ak8963_read_mag(&mag);
		printf("mag x %10.5f, y %10.5f, z %10.5f\n", mag.x, mag.y, mag.z);

		delay_ms(10);
	}

}
