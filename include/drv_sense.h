#ifndef DRV_SENSE_H
#define DRV_SENSE_H

#include <stdint.h>

#include "esp_err.h"

esp_err_t drv_sense_init();

float drv_sense_vsystem();

float drv_sense_vpower();

void drv_sense_test();

#endif // DRV_SENSE_H
