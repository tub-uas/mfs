#include "drv_hmc5883.h"
#include "drv_hmc_regs.h"

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

static i2c_device_t hmc5883;

esp_err_t drv_hmc5883_init() {

	hmc5883.port = 0;
	hmc5883.scl = 32;
	hmc5883.sda = 33;
	hmc5883.addr = HMC5883_ADDR;
	hmc5883.speed = 400000;
	hmc5883.timeout = 1000;

	esp_err_t ret_hmc5883 = drv_i2c_init(hmc5883);

	if (drv_hmc5883_ping()) {
		ESP_LOGE(__FILE__,"HMC5883 not found");
		return ESP_FAIL;
	}

	return ret_hmc5883;
}

esp_err_t drv_hmc5883_ping() {

	uint32_t resp = 0x00;
	drv_i2c_read_bytes(hmc5883, REG_ID_A, 3, (uint8_t*) &resp);

	if (resp != HMC5883_WHO_AM_I_RESP) {
		printf("resp %x \n", resp);
		ESP_LOGE(__FILE__, "HMC5883 not found, ping unsuccessful, WHO_AM_I_RESP does not match");
		return ESP_FAIL;
	}

	return ESP_OK;
}

// esp_err_t drv_hmc5883_config_clk() {
//
// 	// Clear sleep mode
// 	drv_i2c_write_bytes(hmc5883, PWR_MGMT_1, 1, HMC5883_BMASK_CLEAR_PWR1);
// 	delay_ms(100);
//
// 	// Select clock source -> best
// 	drv_i2c_write_bytes(hmc5883, PWR_MGMT_1, 1, HMC5883_BMASK_SELCLK);
// 	delay_ms(100);
//
// 	// Select sample rate -> divider = 4
// 	drv_i2c_write_bytes(hmc5883, SMPLRT_DIV, 1, HMC5883_BMASK_SMPLRT_DIV);
// 	delay_ms(100);
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_set_gyr_filter() {
//
// 	drv_i2c_write_bytes(hmc5883, CONFIG, 1, HMC5883_BMASK_GYR_LPFCFG);
//
// 	uint8_t resp = 0x00;
// 	drv_i2c_read_bytes(hmc5883, GYRO_CONFIG, 1, &resp);
//
// 	resp &= ~(HMC5883_BMASK_GYR_FCHOICEB);
// 	drv_i2c_write_bytes(hmc5883, GYRO_CONFIG, 1, resp);
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_set_acc_filter() {
//
// 	uint8_t resp = 0x00;
// 	drv_i2c_read_bytes(hmc5883, ACCEL_CONFIG_2, 1, &resp);
//
// 	resp &= ~(0x0F);
// 	resp |= HMC5883_BMASK_ACC_LPFCFG;
//
// 	drv_i2c_write_bytes(hmc5883, ACCEL_CONFIG_2, 1, resp);
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_reset() {
//
// 	drv_i2c_write_bytes(hmc5883, PWR_MGMT_1, 1, HMC5883_BMASK_RESET);
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_enable_sensors() {
//
// 	drv_i2c_write_bytes(hmc5883, PWR_MGMT_2, 1, HMC5883_BMASK_EN_ALL);
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_disable_sensors() {
//
// 	drv_i2c_write_bytes(hmc5883, PWR_MGMT_2, 1, HMC5883_BMASK_DIS_ALL);
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_enable_mag_access() {
//
// 	uint8_t data = 0x00;
// 	data &= ~HMC5883_BMASK_I2CMST_DIS;
// 	drv_i2c_write_bytes(hmc5883, USER_CTRL, 1, data);
//
// 	drv_i2c_write_bytes(hmc5883, INT_PIN_CFG, 1, HMC5883_BMASK_BYPASS_EN);
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_set_gyr_scale(hmc5883_gyr_scale_t scale) {
//
// 	uint8_t data;
// 	drv_i2c_read_bytes(hmc5883, GYRO_CONFIG, 1, &data);
//
// 	data &= ~(0x18);
// 	data |= (scale << 3);
//
// 	drv_i2c_write_bytes(hmc5883, GYRO_CONFIG, 1, data);
//
// 	switch(scale) {
// 		case GS_250:
// 			gyr_scale = 250.0/32768.0;
// 			break;
// 		case GS_500:
// 			gyr_scale = 500.0/32768.0;
// 			break;
// 		case GS_1000:
// 			gyr_scale = 1000.0/32768.0;
// 			break;
// 		case GS_2000:
// 			gyr_scale = 2000.0/32768.0;
// 			break;
// 	}
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_set_acc_scale(hmc5883_acc_scale_t scale) {
//
// 	drv_i2c_write_bytes(hmc5883, ACCEL_CONFIG, 1, scale << 3);
//
// 	switch(scale) {
// 		case AS_2:
// 			acc_scale = 2.0/32768.0;
// 			break;
// 		case AS_4:
// 			acc_scale = 4.0/32768.0;
// 			break;
// 		case AS_8:
// 			acc_scale = 8.0/32768.0;
// 			break;
// 		case AS_16:
// 			acc_scale = 16.0/32768.0;
// 			break;
// 	}
//
// 	return ESP_OK;
// }
//
// esp_err_t drv_hmc5883_read_gyr_vec(hmc5883_gyr_data_t *gyr_vec) {
// 	uint8_t raw[6];
// 	drv_i2c_read_bytes(hmc5883, GYRO_XOUT_H, 6, raw);
//
// 	gyr_vec->gyr_y = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]) *  gyr_scale;
// 	gyr_vec->gyr_x = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]) *  gyr_scale;
// 	gyr_vec->gyr_z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]) * -gyr_scale;
//
// 	#if defined(DEFAULT_ORIENT)
// 		// Nothing to be done
// 	#elif defined(HYPE_ORIENT)
// 		float gyr_y_tmp = -gyr_vec->gyr_y;
// 		gyr_vec->gyr_y = gyr_vec->gyr_z;
// 		gyr_vec->gyr_z = gyr_y_tmp;
// 	#else
// 		#error "Unkown board orientation"
// 	#endif
//
// 	return ESP_OK;
// }
//
//
// esp_err_t drv_hmc5883_read_acc_vec(hmc5883_acc_data_t *acc_vec) {
// 	uint8_t raw[6];
// 	drv_i2c_read_bytes(hmc5883, ACCEL_XOUT_H, 6, raw);
//
// 	acc_vec->acc_y = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]) * -acc_scale;
// 	acc_vec->acc_x = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]) * -acc_scale;
// 	acc_vec->acc_z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]) *  acc_scale;
//
// 	#if defined(DEFAULT_ORIENT)
// 		// Nothing to be done
// 	#elif defined(HYPE_ORIENT)
// 		float acc_y_tmp = -acc_vec->acc_y;
// 		acc_vec->acc_y = acc_vec->acc_z;
// 		acc_vec->acc_z = acc_y_tmp;
// 	#else
// 		#error "Unkown board orientation"
// 	#endif
//
// 	return ESP_OK;
// }
//
// float drv_hmc5883_read_temp() {
// 	uint8_t tmp[2] = {0};
// 	drv_i2c_read_bytes(hmc5883, TEMP_OUT_H, 2, tmp);
// 	return (tmp[0]<<8|tmp[1])/100.0;
// }
//
// void drv_hmc5883_test() {
//
// 	ESP_LOGI(__FILE__, "HMC Driver Test");
//
// 	// uint8_t resp = 0x00;
// 	// drv_i2c_read_bytes(hmc5883, WHO_AM_I, 1, &resp);
// 	// if (resp != HMC5883_WHO_AM_I_RESP)
// 	// 	ESP_LOGE(__FILE__, "WHO AM I does not match");
//
// 	while(1) {
//
// 		// uint8_t tmp[2] = {0};
// 		// drv_i2c_read_bytes(hmc5883, TEMP_OUT_H, 2, tmp);
// 		// printf("tmp: %u \n", tmp[0]<<8|tmp[1]);
//
// 		hmc5883_acc_data_t acc;
// 		drv_hmc5883_read_acc_vec(&acc);
// 		printf("acc_x %f, acc_y %f, acc_z %f \n", acc.acc_x, acc.acc_y, acc.acc_z);
//
// 		// hmc5883_gyr_data_t gyr;
// 		// drv_hmc5883_read_gyr_vec(&gyr);
// 		// printf("gyr_x %f, gyr_y %f, gyr_z %f \n", gyr.gyr_x, gyr.gyr_y, gyr.gyr_z);
//
// 		delay_ms(100);
// 	}
//
// }
