#ifndef DRV_MPU9250_H
#define DRV_MPU9250_H

#include <stdint.h>

#include "esp_err.h"

#define MPU9250_ADDR          0x68
#define MPU9250_WHO_AM_I_RESP 0x73
#define MPU9250_TIMEOUT       100

typedef enum {
	GS_250  = 0x00,
	GS_500  = 0x01,
	GS_1000 = 0x02,
	GS_2000 = 0x03,
} mpu9250_gyr_scale_t;

typedef struct {
	float gyr_x;
	float gyr_y;
	float gyr_z;
} mpu9250_gyr_data_t;

typedef enum {
	AS_2  = 0x00,
	AS_4  = 0x01,
	AS_8  = 0x02,
	AS_16 = 0x03,
} mpu9250_acc_scale_t;

typedef struct {
	float acc_x;
	float acc_y;
	float acc_z;
} mpu9250_acc_data_t;

esp_err_t drv_mpu9250_init();

esp_err_t drv_mpu9250_ping();

esp_err_t drv_mpu9250_config_clk();

esp_err_t drv_mpu9250_set_gyr_filter();

esp_err_t drv_mpu9250_set_acc_filter();

esp_err_t drv_mpu9250_reset();

esp_err_t drv_mpu9250_enable_sensors();

esp_err_t drv_mpu9250_disable_sensors();

esp_err_t drv_mpu9250_enable_mag_access();

esp_err_t drv_mpu9250_set_gyr_scale(mpu9250_gyr_scale_t scale);

esp_err_t drv_mpu9250_set_acc_scale(mpu9250_acc_scale_t scale);

esp_err_t drv_mpu9250_read_gyr_vec(mpu9250_gyr_data_t *gyr_vec);

esp_err_t drv_mpu9250_read_acc_vec(mpu9250_acc_data_t *acc_vec);

float drv_mpu9250_read_temp();

void drv_mpu9250_test();

#endif // DRV_MPU9250_H
