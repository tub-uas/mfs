#ifndef DRV_CAN_H
#define DRV_CAN_H

#include <stdint.h>

#include "esp_err.h"

#define CAN_TIMEOUT 500

esp_err_t drv_can_init(uint32_t ids[], uint32_t ids_len);

void drv_can_test();

#endif // DRV_CAN_H
