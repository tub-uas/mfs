#include "main_psu.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "board.h"
#include "drv_button.h"
#include "drv_can.h"
#include "can_com_psu.h"
#include "drv_led.h"
#include "drv_sense_ext.h"
#include "drv_sense.h"
#include "util.h"

void main_psu() {

	ESP_LOGI(__FILE__, "RUNNING MAIN_PSU");

	TickType_t last_wake_time = xTaskGetTickCount();

	drv_led_set(LED_ON_ALIVE);

	while(1) {

		can_com_psu_t data;

		ESP_ERROR_CHECK(drv_sense_ext_read_volt(SENSE_EXT_MAIN_ADDR, &data.sense_main_volt));
		ESP_ERROR_CHECK(drv_sense_ext_read_curr(SENSE_EXT_MAIN_ADDR, &data.sense_main_curr));
		ESP_ERROR_CHECK(drv_sense_ext_read_pow(SENSE_EXT_MAIN_ADDR, &data.sense_main_pow));

		ESP_ERROR_CHECK(drv_sense_ext_read_volt(SENSE_EXT_PWR_ADDR, &data.sense_pwr_volt));
		ESP_ERROR_CHECK(drv_sense_ext_read_curr(SENSE_EXT_PWR_ADDR, &data.sense_pwr_curr));
		ESP_ERROR_CHECK(drv_sense_ext_read_pow(SENSE_EXT_PWR_ADDR, &data.sense_pwr_pow));

		ESP_ERROR_CHECK(drv_sense_ext_read_volt(SENSE_EXT_SYS_ADDR, &data.sense_sys_volt));
		ESP_ERROR_CHECK(drv_sense_ext_read_curr(SENSE_EXT_SYS_ADDR, &data.sense_sys_curr));
		ESP_ERROR_CHECK(drv_sense_ext_read_pow(SENSE_EXT_SYS_ADDR, &data.sense_sys_pow));

		// printf("Main %10f %10f %10f \n", data.sense_main_volt, data.sense_main_curr, data.sense_main_pow);
		// printf("Pwr %10f %10f %10f \n", data.sense_pwr_volt, data.sense_pwr_curr, data.sense_pwr_pow);
		// printf("sys %10f %10f %10f \n", data.sense_sys_volt, data.sense_sys_curr, data.sense_sys_pow);

		// todo: do something more advanced here
		if (data.sense_main_volt < 11.5 ||
		    data.sense_pwr_volt < 4.95 ||
		    data.sense_sys_volt < 4.95) {
			drv_led_set(LED_FAST);
			printf("Main %10f Pwr %10f Sys %10f \n", data.sense_main_volt, data.sense_pwr_volt, data.sense_sys_volt);
		} else {
			drv_led_set(LED_ON_ALIVE);
		}

		can_com_psu_send(data);

		delay_until_ms(&last_wake_time, 100);
	}
}
