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

		// can_com_psu_t psu_data;
		// memset(&psu_data, 0, sizeof(psu_data));
		//
		// // if (can_com_psu_get(&psu_data) == ESP_OK) {
		// // 	// printf("Main %5.2f %5.2f %5.2f   ", psu_data.sense_main_volt, psu_data.sense_main_curr, psu_data.sense_main_pow);
		// // 	// printf("Pwr  %5.2f %5.2f %5.2f   ", psu_data.sense_pwr_volt, psu_data.sense_pwr_curr, psu_data.sense_pwr_pow);
		// // 	// printf("sys  %5.2f %5.2f %5.2f \n", psu_data.sense_sys_volt, psu_data.sense_sys_curr, psu_data.sense_sys_pow);
		// //
		// // } else {
		// // 	ESP_LOGE(__FILE__, "Could not get PSU data from CAN");
		// // }
		// //
		// // mpu9250_acc_data_t acc;
		// // if (drv_mpu9250_read_acc(&acc) != ESP_OK) {
		// // 	ESP_LOGE(__FILE__, "Could not read acc ");
		// // }
		// // printf("acc x %10.5f, y %10.5f, z %10.5f ", acc.x, acc.y, acc.z);
		//
		// mpu9250_gyr_data_t gyr;
		// drv_mpu9250_read_gyr(&gyr);
		// // printf("gyr x %10.5f, y %10.5f, z %10.5f ", gyr.x, gyr.y, gyr.z);
		//
		// ak8963_mag_data_t mag;
		// drv_ak8963_read_mag(&mag) != ESP_OK)
		// // printf("mag x %10.5f, y %10.5f, z %10.5f %10.5f \n", mag.x, mag.y, mag.z, get_time_s());
		//
		// /* Correct mag data for distortion by main current */
		// drv_ak8963_curr_corr(&mag, psu_data.sense_main_curr);
		//
		// // can_com_ahrs_t ahrs_data;
		// // get_attitude(&ahrs_data);
		// //
		// // printf("acc: %10.5f, %10.5f, %10.5f \n", ahrs_data.acc[0], ahrs_data.acc[1], ahrs_data.acc[2]);
		// // printf("att: %10.5f, %10.5f, %10.5f, ", ahrs_data.att[0], ahrs_data.att[1], ahrs_data.att[2]);
		// // printf("temp: %.2f, press: %.2f \n", ahrs_data.temp, ahrs_data.press);
		//
		// // can_com_ahrs_send(ahrs_data);

		// TODO this could potentially send the same AHRS data twice, use notify or smth
		delay_until_ms(&last_wake_time, 10);

	}
}
