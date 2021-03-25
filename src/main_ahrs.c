#include "main_ahrs.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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
#include "can_com_psu.h"
#include "can_com_gps.h"
#include "drv_led.h"
#include "drv_mpu9250.h"
#include "drv_sense.h"
#include "util.h"

void main_ahrs() {

	ESP_LOGI(__FILE__, "RUNNING MAIN_AHRS");

	TickType_t last_wake_time = xTaskGetTickCount();

	// TODO LED is controlled by attitude worker for now (should actually happen here)
	// drv_led_set(LED_ON_ALIVE);

	while(1) {
		
		can_com_ahrs_t ahrs_data;

		/* Get data */
		get_attitude(&ahrs_data);

		/* and send it over CAN */
		can_com_ahrs_send(ahrs_data);

		// TODO this could potentially send the same AHRS data twice, use notify or smth
		delay_until_ms(&last_wake_time, 10);

	}
}
