#ifndef DRV_HMC5883_H
#define DRV_HMC5883_H

#include <stdint.h>

#include "esp_err.h"

#define HMC5883_ADDR          0x1E
#define HMC5883_WHO_AM_I_RESP 0x00333448
#define HMC5883_TIMEOUT       100

// typedef enum {
// 	GS_250  = 0x00,
// 	GS_500  = 0x01,
// 	GS_1000 = 0x02,
// 	GS_2000 = 0x03,
// } hmc5883_gyr_scale_t;
//
// typedef struct {
// 	float gyr_x;
// 	float gyr_y;
// 	float gyr_z;
// } hmc5883_gyr_data_t;
//
// typedef enum {
// 	AS_2  = 0x00,
// 	AS_4  = 0x01,
// 	AS_8  = 0x02,
// 	AS_16 = 0x03,
// } hmc5883_acc_scale_t;
//
// typedef struct {
// 	float acc_x;
// 	float acc_y;
// 	float acc_z;
// } hmc5883_acc_data_t;

esp_err_t drv_hmc5883_init();

esp_err_t drv_hmc5883_ping();

// esp_err_t drv_hmc5883_config_clk();
//
// esp_err_t drv_hmc5883_set_gyr_filter();
//
// esp_err_t drv_hmc5883_set_acc_filter();
//
// esp_err_t drv_hmc5883_reset();
//
// esp_err_t drv_hmc5883_enable_sensors();
//
// esp_err_t drv_hmc5883_disable_sensors();
//
// esp_err_t drv_hmc5883_enable_mag_access();
//
// esp_err_t drv_hmc5883_set_gyr_scale(hmc5883_gyr_scale_t scale);
//
// esp_err_t drv_hmc5883_set_acc_scale(hmc5883_acc_scale_t scale);
//
// esp_err_t drv_hmc5883_read_gyr_vec(hmc5883_gyr_data_t *gyr_vec);
//
// esp_err_t drv_hmc5883_read_acc_vec(hmc5883_acc_data_t *acc_vec);
//
// float drv_hmc5883_read_temp();
//
// void drv_hmc5883_test();

#endif // DRV_HMC5883_H
