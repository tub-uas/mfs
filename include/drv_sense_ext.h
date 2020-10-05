#ifndef DRV_SENSE_EXT_H
#define DRV_SENSE_EXT_H

#include <stdint.h>

#include "esp_err.h"

#define SENSE_EXT_MAIN_ADDR  0x40
#define SENSE_EXT_PWR_ADDR   0x41
#define SENSE_EXT_SYS_ADDR   0x44

esp_err_t drv_sense_ext_init();

esp_err_t drv_sense_ext_read_volt(uint8_t addr, float *volt);

esp_err_t drv_sense_ext_read_curr(uint8_t addr, float *curr);

esp_err_t drv_sense_ext_read_pow(uint8_t addr, float *pow);

void drv_sense_ext_test();

#endif // DRV_SENSE_EXT_H
