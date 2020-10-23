#ifndef DRV_I2C_H
#define DRV_I2C_H

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
	uint8_t    port;
	gpio_num_t scl;
	gpio_num_t sda;
	uint8_t    addr;
	uint32_t   speed;
	uint32_t   timeout;
} i2c_device_t;

esp_err_t drv_i2c_init(i2c_device_t dev);

esp_err_t drv_i2c_read_bytes(i2c_device_t dev, uint8_t reg, uint8_t len, uint8_t *val);

esp_err_t drv_i2c_write_bytes(i2c_device_t dev, uint8_t reg, uint8_t len, uint8_t val);

esp_err_t drv_i2c_update_byte(i2c_device_t dev, uint8_t reg, uint8_t mask, uint8_t val);


#endif // DRV_I2C_H
