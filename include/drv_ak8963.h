#ifndef DRV_AK8963_H
#define DRV_AK8963_H

#include <stdint.h>

#include "esp_err.h"

#define AK8963_ADDR          0x0C
#define AK8963_WHO_AM_I_RESP 0x48
#define AK8963_TIMEOUT       100

typedef enum {
	PWRDOWN     = 0x00,
	SINGLEMEAS  = 0x01,
	CONTINUOUS1 = 0x02,
	CONTINUOUS2 = 0x06,
	EXTTRIG     = 0x04,
	SELFTEST    = 0x08,
	FUSEROMACS  = 0x0F,
} ak8963_mode_t;

typedef enum {
	BITOUT14 = 0x00,
	BITOUT16 = 0x01,
} ak8963_bit_t;

typedef struct {
	uint8_t asa_x;
	uint8_t asa_y;
	uint8_t asa_z;
	float adj_x;
	float adj_y;
	float adj_z;
} ak8963_sens_coeff_t;

typedef struct {
	float x;
	float y;
	float z;
} __attribute__((packed)) ak8963_mag_data_t;

esp_err_t drv_ak8963_init();

esp_err_t drv_ak8963_set_power_down();

esp_err_t drv_ak8963_set_cntl1(ak8963_mode_t mode, ak8963_bit_t bit);

esp_err_t drv_ak8963_ping();

esp_err_t drv_ak8963_read_sens_coeff();

esp_err_t ak8963_reset();

uint8_t drv_ak8963_data_rdy();

uint8_t drv_ak8963_status();

esp_err_t drv_ak8963_read_mag_native(ak8963_mag_data_t *mag);

esp_err_t drv_ak8963_read_mag(ak8963_mag_data_t *mag);

void drv_ak8963_test();

#endif // DRV_AK8963_H
