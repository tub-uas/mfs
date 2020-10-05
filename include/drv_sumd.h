#ifndef DRV_SUMD_H
#define DRV_SUMD_H

#include <stdint.h>

#include "esp_err.h"

#define SUMD_CHNL_NUM 12
// #define SUMD_CHNL_NUM 8

esp_err_t drv_sumd_init();

esp_err_t drv_sumd_get_pwm(uint16_t pwm[]);

void drv_sumd_test();

#endif // DRV_SUMD_H
