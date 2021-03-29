#include "drv_bmp280.h"
#include "drv_bmp_regs.h"

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

static i2c_device_t bmp280;
static bmp280_calib_t bmp280_calib;

esp_err_t drv_bmp280_init() {

	bmp280.port = 0;
	bmp280.scl = 32;
	bmp280.sda = 33;
	bmp280.addr = BMP280_ADDR;
	bmp280.speed = 400000;
	bmp280.timeout = 1000;

	bmp280_init_t bmp280_init = {
		.pwr_mode   = MODE_NORMAL,
		.stdby_time = MS01250,
		.ovrs_temp  = TSMPLE_X1,
		.ovrs_press = PSMPLE_X16,
		.filter_set = FILTOFF,
	};

	// !!! Already initalized by MPU9250 -> call mpu9250_init() before bmp280_init() !!!
	esp_err_t ret_bmp280 = ESP_OK; // = drv_i2c_init(bmp280);

	ret_bmp280 |= drv_bmp280_reset();

	delay_ms(1000);

	if (drv_bmp280_ping()) {
		ESP_LOGE(__FILE__,"BMP280 not found");
		return ESP_FAIL;
	}

	ret_bmp280 |= drv_bmp280_set_config_reg(bmp280_init);
	ret_bmp280 |= drv_bmp280_set_ctrl_meas_reg(bmp280_init);
	ret_bmp280 |= drv_bmp280_read_coeff(&bmp280_calib);

	return ret_bmp280;
}

esp_err_t drv_bmp280_ping() {
	uint8_t resp;
	drv_i2c_read_bytes(bmp280, BMP280_WHO_AM_I, 1, &resp);
	if (resp != BMP280_WHO_AM_I_RESP) {
		ESP_LOGE(__FILE__, "BMP280 not found, ping unsuccessful, WHO_AM_I_RESP does not match");
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t drv_bmp280_reset() {
	drv_i2c_write_bytes(bmp280, BMP280_RESET, 1, 0xB6);
	return ESP_OK;
}

esp_err_t drv_bmp280_set_filter(bmp280_filter_t filt) {
	uint8_t data = 0x00;
	drv_i2c_read_bytes(bmp280, BMP280_CONFIG, 1, &data);
	data |= filt<<2;
	drv_i2c_write_bytes(bmp280, BMP280_CONFIG, 1, data);
	return ESP_OK;
}

esp_err_t drv_bmp280_set_ovrs_press(bmp280_ovrs_press_t ovrs_press) {
	uint8_t data = 0x00;
	drv_i2c_read_bytes(bmp280, BMP280_CTRL_MEAS, 1, &data);
	data |= ovrs_press<<2;
	drv_i2c_write_bytes(bmp280, BMP280_CTRL_MEAS, 1, data);
	return ESP_OK;
}

esp_err_t drv_bmp280_set_ovrs_temp(bmp280_ovrs_temp_t ovrs_temp) {
	uint8_t data = 0x00;
	drv_i2c_read_bytes(bmp280, BMP280_CTRL_MEAS, 1, &data);
	data |= ovrs_temp<<5;
	drv_i2c_write_bytes(bmp280, BMP280_CTRL_MEAS, 1, data);
	return ESP_OK;
}

esp_err_t drv_bmp280_set_mode(bmp280_pwrm_t mode) {
	uint8_t data = 0x00;
	drv_i2c_read_bytes(bmp280, BMP280_CTRL_MEAS, 1, &data);
	data |= mode;
	drv_i2c_write_bytes(bmp280, BMP280_CTRL_MEAS, 1, data);
	return ESP_OK;
}

esp_err_t drv_bmp280_set_standby_duration(bmp280_stdby_t stdby) {
	uint8_t data = 0x00;
	drv_i2c_read_bytes(bmp280, BMP280_CTRL_MEAS, 1, &data);
	data |= stdby<<5;
	drv_i2c_write_bytes(bmp280, BMP280_CTRL_MEAS, 1, data);
	return ESP_OK;
}

esp_err_t drv_bmp280_set_config_reg(bmp280_init_t init) {
	uint8_t data = 0;
	data |= init.stdby_time<<5;
	data |= init.filter_set<<2;
	drv_i2c_write_bytes(bmp280, BMP280_CONFIG, 1, data);
	return ESP_OK;
}

esp_err_t drv_bmp280_set_ctrl_meas_reg(bmp280_init_t init) {
	uint8_t data = 0;
	data |= init.ovrs_temp<<5;
	data |= init.ovrs_press<<2;
	data |= init.pwr_mode;
	drv_i2c_write_bytes(bmp280, BMP280_CTRL_MEAS, 1, data);
	return ESP_OK;
}

esp_err_t drv_bmp280_read_coeff(bmp280_calib_t *cal) {

	uint8_t read[2] = {0x00, 0x00};
	drv_i2c_read_bytes(bmp280, BMP280_REGT1, SIZECOEFFREG, read);
	cal->dig_T1 = (uint16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGT2, SIZECOEFFREG, read);
	cal->dig_T2 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGT3, SIZECOEFFREG, read);
	cal->dig_T3 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP1, SIZECOEFFREG, read);
	cal->dig_P1 = (uint16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP2, SIZECOEFFREG, read);
	cal->dig_P2 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP3, SIZECOEFFREG, read);
	cal->dig_P3 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP4, SIZECOEFFREG, read);
	cal->dig_P4 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP5, SIZECOEFFREG, read);
	cal->dig_P5 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP6, SIZECOEFFREG, read);
	cal->dig_P6 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP7, SIZECOEFFREG, read);
	cal->dig_P7 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP8, SIZECOEFFREG, read);
	cal->dig_P8 = (int16_t)(read[1]<<8) | read[0];

	drv_i2c_read_bytes(bmp280, BMP280_REGP9, SIZECOEFFREG, read);
	cal->dig_P9 = (int16_t)(read[1]<<8) | read[0];

	return ESP_OK;
}

/* Returns the temperature in °c (degree celsius) */
double bmp280_get_temp() {

	static int32_t temp_calib;
	return bmp280_get_temp_raw(&temp_calib, bmp280_calib);
}

/* Returns the ambient pressure in ?? */
double bmp280_get_press() {

	static int32_t temp_calib;
	bmp280_get_temp_raw(&temp_calib, bmp280_calib);
	return bmp280_get_press_raw(&temp_calib, bmp280_calib);
}

double bmp280_get_temp_raw(int32_t *t_data, bmp280_calib_t cal) {

	double var1, var2, T;
	uint8_t tmp[SIZETEMPREG] = {0x00};

	uint32_t test = 0;
	drv_i2c_read_bytes(bmp280, BMP280_TEMP_HB, SIZETEMPREG, tmp);
	test = tmp[0];
	test <<= 8;
	test |= tmp[1];
	test <<= 8;
	test |= tmp[2];
	test >>= 4;
	test &= 0xFFFFF;
	int32_t adc_T = (int32_t)test;

	var1 = (((double)adc_T)/16384.0 - ((double)cal.dig_T1)/1024.0) * ((double)cal.dig_T2);

	var2 = ((((double)adc_T)/131072.0 - ((double)cal.dig_T1)/8192.0) *
	       (((double)adc_T)/131072.0 - ((double)cal.dig_T1)/8192.0)) * ((double)cal.dig_T3);

	*t_data = (int32_t) (var1 + var2);

	T = (var1 + var2) / 5120.0;
	return T;
}

double bmp280_get_press_raw(int32_t *t_data, bmp280_calib_t cal) {

	double var3, var4, p;

	uint8_t tmp[SIZEPRESSREG] = {0x00};
	uint32_t test = 0;
	drv_i2c_read_bytes(bmp280, BMP280_PRESS_HB, SIZEPRESSREG, tmp);
	test = (uint32_t)(((uint32_t)tmp[0]<<8|tmp[1])<<8|tmp[2]);
	test >>= 4;

	int32_t adc_P = (int32_t)test;

	var3 = ((double) *t_data/2.0) - 64000.0;
	var4 = var3 * var3 * (double)cal.dig_P6/32768.0;
	var4 = var4 + ((var3 * (double)cal.dig_P5) * 2.0);
	var4 = var4 + (((double)cal.dig_P4) * 65536.0);
	var3 = (((double)cal.dig_P3) * var3 * var3 / 524288.0 + ((double)cal.dig_P2)*var3) / 524288.0;
	var3 = (1.0 +var3 / 32768.0) * ((double)cal.dig_P1);

	if (var3 == 0.0) {
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576.0 - (double)adc_P;
	p = (p - (var4 / 4096.0)) * 6250.0/var3;
	var3 = ((double)cal.dig_P9) * p * p / 2147483648.0;
	var4 = p * ((double)cal.dig_P8) / 32768.0;
	p = p + (var3 + var4 + ((double)cal.dig_P7)) / 16.0;
	return p;
}

void drv_bmp280_test() {

	ESP_LOGI(__FILE__, "BMP Driver Test");

	while(1) {

		printf("nPress: %.2f, Temp: %.2f \n", bmp280_get_press(),
		                                      bmp280_get_temp());
		delay_ms(100);
	}

}
