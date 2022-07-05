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

static const float gain_values [] = {
	[HMC5883L_GAIN_1370] = 0.73,
	[HMC5883L_GAIN_1090] = 0.92,
	[HMC5883L_GAIN_820]  = 1.22,
	[HMC5883L_GAIN_660]  = 1.52,
	[HMC5883L_GAIN_440]  = 2.27,
	[HMC5883L_GAIN_390]  = 2.56,
	[HMC5883L_GAIN_330]  = 3.03,
	[HMC5883L_GAIN_230]  = 4.35
};

static const double hmc5883_softiron[3][3] = {
	{1.04767,-7.1446e-3,1.9039e-2},
	{-7.1446e-3,9.852e-1,-6.2472e-2},
	{1.9039e-2,-6.2472e-2,9.4355e-1},
};

static const double hmc5883_hardiron[3] = {
	6.5391,
	-1.5162e1,
	-3.8937,
};

static i2c_device_t hmc5883;
static drv_hmc5883_opmode_t opmode;
float gain = gain_values[HMC5883L_GAIN_1090];

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
	uint8_t val = 0;
	drv_i2c_read_bytes(hmc5883, REG_MODE, 1, &val);

	ESP_LOGI(__FILE__, "HMC5883 mode: %x \n", val);

	// drv_hmc5883_gain_t gain_tmp = 0;
	// drv_hmc5883_get_gain(&gain_tmp);
	// gain = gain_values[gain_tmp];
	//
	// drv_hmc5883_get_opmode(&opmode);
	//
	// printf("gain: %f \n", gain);
	// printf("opmode: %d \n", opmode);
	//
	// drv_hmc5883_set_opmode(HMC5883L_MODE_SINGLE);
	//
	// drv_hmc5883_get_opmode(&opmode);
	//
	// printf("gain: %f \n", gain);
	// printf("opmode2: %d \n", opmode);

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

esp_err_t drv_hmc5883_get_opmode(drv_hmc5883_opmode_t *val) {

	drv_i2c_read_bytes(hmc5883, REG_MODE, 1, (uint8_t *)val);

	printf("HMC5883 opmode raw %d \n", *val & MASK_MD);

	*val = (*val & MASK_MD) == 0 ? HMC5883L_MODE_CONTINUOUS : HMC5883L_MODE_SINGLE;

	return ESP_OK;
}

esp_err_t drv_hmc5883_set_opmode(drv_hmc5883_opmode_t mode) {

	drv_i2c_write_bytes(hmc5883, REG_MODE, 1, mode);

	opmode = mode;

	return ESP_OK;
}

esp_err_t drv_hmc5883_get_samples_averaged(drv_hmc5883_samples_averaged_t *val) {

	drv_i2c_read_bytes(hmc5883, REG_CR_A, 1, (uint8_t *)val);

	*val = (*val & MASK_MA) >> BIT_MA;

	return ESP_OK;
}

esp_err_t drv_hmc5883_set_samples_averaged(drv_hmc5883_samples_averaged_t samples) {

	drv_i2c_update_byte(hmc5883, REG_CR_A, MASK_MA, samples << BIT_MA);

	return ESP_OK;
}

esp_err_t drv_hmc5883_get_data_rate(drv_hmc5883_data_rate_t *val) {

	drv_i2c_read_bytes(hmc5883, REG_CR_A, 1, (uint8_t *)val);

	*val = (*val & MASK_DO) >> BIT_DO;
	return ESP_OK;
}

esp_err_t drv_hmc5883_set_data_rate(drv_hmc5883_data_rate_t rate) {

	drv_i2c_update_byte(hmc5883, REG_CR_A, MASK_DO, rate << BIT_DO);

	return ESP_OK;
}

esp_err_t drv_hmc5883_get_bias(drv_hmc5883_bias_t *val) {

	drv_i2c_read_bytes(hmc5883, REG_CR_A, 1, (uint8_t *)val);

	*val &= MASK_MS;
	return ESP_OK;
}

esp_err_t drv_hmc5883_set_bias(drv_hmc5883_bias_t bias) {

	drv_i2c_update_byte(hmc5883, REG_CR_A, MASK_MS, bias);

	return ESP_OK;
}

esp_err_t drv_hmc5883_get_gain(drv_hmc5883_gain_t *val) {

	drv_i2c_read_bytes(hmc5883, REG_CR_B, 1, (uint8_t *)val);

	*val >>= BIT_GN;
	return ESP_OK;
}

esp_err_t drv_hmc5883_set_gain(drv_hmc5883_gain_t gain) {
	drv_i2c_write_bytes(hmc5883, REG_CR_B, 1, gain << BIT_GN);

	gain = gain_values[gain];
	return ESP_OK;
}

esp_err_t drv_hmc5883_data_is_locked(uint8_t *val) {

	drv_i2c_read_bytes(hmc5883, REG_STAT, 1, (uint8_t *)val);

	*val &= MASK_DL;
	return ESP_OK;
}

esp_err_t drv_hmc5883_data_is_ready(uint8_t *val) {

	drv_i2c_read_bytes(hmc5883, REG_STAT, 1, (uint8_t *)val);

	*val &= MASK_DR;
	return ESP_OK;
}

esp_err_t drv_hmc5883_get_raw_data(drv_hmc5883_raw_data_t *data) {

	uint8_t buf[6];

	drv_i2c_read_bytes(hmc5883, REG_DX_H, 6, buf);

	data->x = ((int16_t)buf[REG_DX_H - REG_DX_H] << 8) | buf[REG_DX_L - REG_DX_H];
	data->y = ((int16_t)buf[REG_DY_H - REG_DX_H] << 8) | buf[REG_DY_L - REG_DX_H];
	data->z = ((int16_t)buf[REG_DZ_H - REG_DX_H] << 8) | buf[REG_DZ_L - REG_DX_H];

	return ESP_OK;
}

/* Raw to microtesla */
esp_err_t drv_hmc5883_raw_to_mt(drv_hmc5883_raw_data_t raw, drv_hmc5883_data_t *mt) {
	mt->x = raw.x * gain * 0.1;
	mt->y = raw.y * gain * 0.1;
	mt->z = raw.z * gain * 0.1;

	return ESP_OK;
}

/* Return magnetometer data in microtesla */
esp_err_t drv_hmc5883_get_data(drv_hmc5883_data_t *data) {

	drv_hmc5883_raw_data_t raw;

	esp_err_t ret = drv_hmc5883_get_raw_data(&raw);
	drv_hmc5883_raw_to_mt(raw, data);
	
	// TRANSFORM ORIENTATION FROM HYPE WING
	// drv_hmc5883_correct_data(data);
	float tmp_x = data->x*0.7071, tmp_y = data->y*0.7071; 
	data->x = -tmp_y+tmp_x;
	data->y = tmp_x+tmp_y;
	
	// data->z = data->z; // no necessary computation

	return ret;
}

esp_err_t drv_hmc5883_correct_data(drv_hmc5883_data_t *data) {

	double tmp_0 = data->x + hmc5883_hardiron[0];
	double tmp_1 = data->y + hmc5883_hardiron[1];
	double tmp_2 = data->z + hmc5883_hardiron[2];
	double out[3] = {0.0f};

	out[0] = hmc5883_softiron[0][0] * tmp_0 + hmc5883_softiron[0][1]* tmp_1 + hmc5883_softiron[0][2] * tmp_2;
	out[1] = hmc5883_softiron[1][0] * tmp_0 + hmc5883_softiron[1][1]* tmp_1 + hmc5883_softiron[1][2] * tmp_2;
	out[2] = hmc5883_softiron[2][0] * tmp_0 + hmc5883_softiron[2][1]* tmp_1 + hmc5883_softiron[2][2] * tmp_2;
	data->x = out[0];
	data->y = out[1];
	data->z = out[2];

	return ESP_OK;
}