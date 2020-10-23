#include "drv_i2c.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "util.h"

esp_err_t drv_i2c_init(i2c_device_t dev) {
	ESP_LOGI(__FILE__, "Init I2C device, address: %02x \n", dev.addr);

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.scl_io_num = dev.scl;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.sda_io_num = dev.sda;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = dev.speed;
	esp_err_t ret_i2c = i2c_param_config(dev.port, &conf);

	/* Start the I2C Driver */
	ret_i2c |= i2c_driver_install(dev.port, conf.mode, 0, 0, 0);
	if (ret_i2c != ESP_OK)
		ESP_LOGE(__FILE__, "Cant start I2C Driver");

	return ESP_OK;
}

esp_err_t drv_i2c_read_bytes(i2c_device_t dev, uint8_t reg, uint8_t len, uint8_t *val) {

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(cmd, val, len, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

	esp_err_t ret = i2c_master_cmd_begin(dev.port,
	                                     cmd,
	                                     dev.timeout / portTICK_RATE_MS);

	if (ret != ESP_OK)
		ESP_LOGW(__FILE__, "Cant read I2C Register ");

	i2c_cmd_link_delete(cmd);

	return ret;
}

esp_err_t drv_i2c_write_bytes(i2c_device_t dev, uint8_t reg, uint8_t len, uint8_t val) {

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write(cmd, &val, len, true);
	i2c_master_stop(cmd);

	esp_err_t ret = i2c_master_cmd_begin(dev.port,
	                                     cmd,
	                                     dev.timeout / portTICK_RATE_MS);

	if (ret != ESP_OK)
		ESP_LOGW(__FILE__, "Cant write I2C Register");

	i2c_cmd_link_delete(cmd);

	return ret;
}

esp_err_t drv_i2c_update_byte(i2c_device_t dev, uint8_t reg, uint8_t mask, uint8_t val) {

	uint8_t old;
	esp_err_t ret = drv_i2c_read_bytes(dev, reg, 1, &old);

	if (ret != ESP_OK)
		return ret;
	else
		return drv_i2c_write_bytes(dev, reg, 1, (old & mask) | val);
}
