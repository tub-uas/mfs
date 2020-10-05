#ifndef DRV_BUTTON_H
#define DRV_BUTTON_H

#include <stdint.h>

#include "esp_err.h"

typedef enum {
	BUTTON_NOT_PRESSED = 0,
	BUTTON_PRESSED
} button_status_t;

esp_err_t drv_button_init();

uint32_t drv_button_pressed();

void drv_button_test();

#endif // DRV_BUTTON_H
