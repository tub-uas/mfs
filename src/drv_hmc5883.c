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
hmc5883l_opmode_t opmode;
float gain;

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

esp_err_t hmc5883l_init(hmc5883l_dev_t *dev)
{
	CHECK_ARG(dev);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

	uint32_t id = 0;
	I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, REG_ID_A, &id, 3));
	if (id != HMC5883L_ID)
	{
		I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
		ESP_LOGE(TAG, "Unknown ID: 0x%08x != 0x%08x", id, HMC5883L_ID);
		return ESP_ERR_NOT_FOUND;
	}

	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	hmc5883l_gain_t gain;
	hmc5883l_get_gain(dev, &gain)
	dev->gain = gain_values[gain];

	CHECK(hmc5883l_get_opmode(dev, &dev->opmode));

	return ESP_OK;
}

esp_err_t hmc5883l_get_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t *val)
{
	CHECK_ARG(dev && val);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_MODE, (uint8_t *)val));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	*val = (*val & MASK_MD) == 0 ? HMC5883L_MODE_CONTINUOUS : HMC5883L_MODE_SINGLE;
	return ESP_OK;
}

esp_err_t hmc5883l_set_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t mode)
{
	CHECK_ARG(dev);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, REG_MODE, mode));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	dev->opmode = mode;
	return ESP_OK;
}

esp_err_t hmc5883l_get_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t *val)
{
	CHECK_ARG(dev && val);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_A, (uint8_t *)val));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	*val = (*val & MASK_MA) >> BIT_MA;
	return ESP_OK;
}

esp_err_t hmc5883l_set_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t samples)
{
	CHECK_ARG(dev);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, update_register(dev, REG_CR_A, MASK_MA, samples << BIT_MA));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return ESP_OK;
}

esp_err_t hmc5883l_get_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t *val)
{
	CHECK_ARG(dev && val);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_A, (uint8_t *)val));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	*val = (*val & MASK_DO) >> BIT_DO;
	return ESP_OK;
}

esp_err_t hmc5883l_set_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t rate)
{
	CHECK_ARG(dev);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, update_register(dev, REG_CR_A, MASK_DO, rate << BIT_DO));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return ESP_OK;
}

esp_err_t hmc5883l_get_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t *val)
{
	CHECK_ARG(dev && val);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_A, (uint8_t *)val));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	*val &= MASK_MS;
	return ESP_OK;
}

esp_err_t hmc5883l_set_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t bias)
{
	CHECK_ARG(dev);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, update_register(dev, REG_CR_A, MASK_MS, bias));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	return ESP_OK;
}

esp_err_t hmc5883l_get_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t *val)
{
	CHECK_ARG(dev && val);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_CR_B, (uint8_t *)val));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	*val >>= BIT_GN;
	return ESP_OK;
}

esp_err_t hmc5883l_set_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t gain)
{
	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, REG_CR_B, gain << BIT_GN));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	dev->gain = gain_values[gain];
	return ESP_OK;
}

esp_err_t hmc5883l_data_is_locked(hmc5883l_dev_t *dev, bool *val)
{
	CHECK_ARG(dev && val);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_STAT, (uint8_t *)val));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	*val &= MASK_DL;
	return ESP_OK;
}

esp_err_t hmc5883l_data_is_ready(hmc5883l_dev_t *dev, bool *val)
{
	CHECK_ARG(dev && val);

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, REG_STAT, (uint8_t *)val));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	*val &= MASK_DR;
	return ESP_OK;
}

esp_err_t hmc5883l_get_raw_data(hmc5883l_dev_t *dev, hmc5883l_raw_data_t *data)
{
	CHECK_ARG(dev && data);

	if (dev->opmode == HMC5883L_MODE_SINGLE)
	{
		// overwrite mode register for measurement
		CHECK(hmc5883l_set_opmode(dev, dev->opmode));
		// wait for data
		uint64_t start = esp_timer_get_time();
		bool dready = false;
		do
		{
			CHECK(hmc5883l_data_is_ready(dev, &dready));
			if (timeout_expired(start, CONFIG_HMC5883L_MEAS_TIMEOUT))
				return ESP_ERR_TIMEOUT;
		} while (!dready);
	}
	uint8_t buf[6];
	uint8_t reg = REG_DX_H;

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
	I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, buf, 6));
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	data->x = ((int16_t)buf[REG_DX_H - REG_DX_H] << 8) | buf[REG_DX_L - REG_DX_H];
	data->y = ((int16_t)buf[REG_DY_H - REG_DX_H] << 8) | buf[REG_DY_L - REG_DX_H];
	data->z = ((int16_t)buf[REG_DZ_H - REG_DX_H] << 8) | buf[REG_DZ_L - REG_DX_H];

	return ESP_OK;
}

esp_err_t hmc5883l_raw_to_mg(const hmc5883l_dev_t *dev, const hmc5883l_raw_data_t *raw, hmc5883l_data_t *mg)
{
	CHECK_ARG(dev && raw && mg);

	mg->x = raw->x * dev->gain;
	mg->y = raw->y * dev->gain;
	mg->z = raw->z * dev->gain;

	return ESP_OK;
}

esp_err_t hmc5883l_get_data(hmc5883l_dev_t *dev, hmc5883l_data_t *data)
{
	CHECK_ARG(data);

	hmc5883l_raw_data_t raw;

	CHECK(hmc5883l_get_raw_data(dev, &raw));
	CHECK(hmc5883l_raw_to_mg(dev, &raw, data));

	return ESP_OK;
}
