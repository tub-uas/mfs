#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define PI 3.14159265358f

esp_err_t delay_us(uint32_t us);

esp_err_t delay_ms(uint32_t ms);

esp_err_t delay_until_ms(TickType_t *last_wake_time, uint32_t ms);

float get_time_s_highres();

uint64_t get_time_us();

uint32_t get_time_ms();

float get_time_s();

float mean(float vals[], int len);

float var(float vals[], int len);

float stde(float vals[], int len);


#endif // UTIL_H
