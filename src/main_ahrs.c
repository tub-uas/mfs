#include "main_ahrs.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "attitude_estimation.h"
#include "board.h"
#include "drv_ak8963.h"
#include "drv_bmp280.h"
#include "drv_button.h"
#include "drv_can.h"
#include "can_com_ahrs.h"
#include "drv_led.h"
#include "drv_mpu9250.h"
#include "drv_sense.h"
#include "util.h"

void main_ahrs() {

	ESP_LOGI(__FILE__, "RUNNING MAIN_AHRS");

	TickType_t last_wake_time = xTaskGetTickCount();

	// todo: LED is controlled by attitude worker for now (should actually happen here)
	// drv_led_set(LED_ON_ALIVE);

	while(1) {

		// drv_led_msg(LED_ON);
		// delay_ms(500);
		// drv_led_msg(LED_OFF);
		// delay_ms(500);

		can_com_ahrs_t ahrs_data;
		get_attitude(&ahrs_data);

		printf("att: %10.5f, %10.5f, %10.5f, ", ahrs_data.att[0], ahrs_data.att[1], ahrs_data.att[2]);
		printf("temp: %.2f, press: %.2f \n", ahrs_data.temp, ahrs_data.press);

		can_com_ahrs_send(ahrs_data);

		// TODO this could potentially send the same AHRS data twice, use notify or smth
		delay_until_ms(&last_wake_time, 10);
		// delay_ms(10);

	}
}
