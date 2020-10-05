#ifndef DRV_LED_H
#define DRV_LED_H

#include <stdint.h>

#include "esp_err.h"

typedef enum {
	LED_OFF = 0,
	LED_SLOW,
	LED_MEDIUM,
	LED_FAST,
	LED_ON_ALIVE,
	LED_ON,
} led_state_t;

esp_err_t drv_led_init();

esp_err_t drv_led_set(led_state_t state);

void drv_led_worker();

void drv_led_test();

#endif // DRV_LED_H
