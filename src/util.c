#include "util.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#define SHORT_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
#define SHORT_BINARY(num)  \
	(num & 0x8000 ? '1' : '0'), \
	(num & 0x4000 ? '1' : '0'), \
	(num & 0x2000 ? '1' : '0'), \
	(num & 0x1000 ? '1' : '0'), \
	(num & 0x0800 ? '1' : '0'), \
	(num & 0x0400 ? '1' : '0'), \
	(num & 0x0200 ? '1' : '0'), \
	(num & 0x0100 ? '1' : '0'), \
	(num & 0x0080 ? '1' : '0'), \
	(num & 0x0040 ? '1' : '0'), \
	(num & 0x0020 ? '1' : '0'), \
	(num & 0x0010 ? '1' : '0'), \
	(num & 0x0008 ? '1' : '0'), \
	(num & 0x0004 ? '1' : '0'), \
	(num & 0x0002 ? '1' : '0'), \
	(num & 0x0001 ? '1' : '0')

esp_err_t delay_us(uint32_t us) {
	usleep(us);
	return ESP_OK;
}

esp_err_t delay_ms(uint32_t ms) {
	uint32_t ticks = ms / portTICK_PERIOD_MS;
	vTaskDelay(ticks);
	return ESP_OK;
}

esp_err_t delay_until_ms(TickType_t *last_wake_time, uint32_t ms) {
	vTaskDelayUntil(last_wake_time, ms/portTICK_PERIOD_MS);
	return ESP_OK;
}

uint64_t get_time_us() {
	return esp_timer_get_time();
}

float get_time_s_highres() {
	return esp_timer_get_time() / 1.0e6;
}

uint32_t get_time_ms() {
	return xTaskGetTickCount() / portTICK_PERIOD_MS;
}

float get_time_s() {
	return xTaskGetTickCount() / (portTICK_PERIOD_MS*1000.0);
}
