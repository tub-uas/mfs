#include "main_gps.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "drv_button.h"
#include "drv_can.h"
#include "drv_led.h"
#include "drv_sense.h"
#include "util.h"


void main_gps() {

	ESP_LOGI(__FILE__, "Running main_gps");

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {

		drv_led_set(LED_ON_ALIVE);

		printf("hello biatch %f \n", get_time_s());

		delay_until_ms(&last_wake_time, 10);
	}

	// If we get here, something went badly wrong. Reset the system
	// TODO set the failsafe mode
	esp_restart();
}
