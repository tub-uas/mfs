#include "main_rai.h"

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
#include "can_com_rai.h"
#include "drv_led.h"
#include "drv_pwm.h"
#include "drv_sense.h"
#include "drv_sumd.h"
#include "util.h"


void send_rai() {

	ESP_LOGI(__FILE__, "Running send_rai");

	TickType_t last_wake_time = xTaskGetTickCount();

	// TODO LED is controlled by recv worker for now (should also happen here)
	// drv_led_set(LED_ON_ALIVE);

	while (1) {

		uint16_t pwm[SUMD_CHNL_NUM] = {0};
		if (drv_sumd_get_pwm(pwm) == ESP_OK) {

			// for (uint32_t i=0; i<SUMD_CHNL_NUM; i++) {
			// 	printf("%d:%d  ", i, pwm[i]);
			// }
			// printf("\n");

			can_com_rai_t data;
			memcpy(&data.chnl[0], pwm, SUMD_CHNL_NUM * sizeof(uint16_t));
			data.time = get_time_s();

			can_com_rai_send(data);

		} else {
			ESP_LOGE(__FILE__, "Send RAI could not get valid PWM data");
		}

		delay_until_ms(&last_wake_time, 10);
	}

	// If we get here, something went badly wrong. Reset the system
	// TODO set the failsafe mode
	esp_restart();
}

void recv_rai() {

	ESP_LOGI(__FILE__, "Running recv_rai");

	drv_led_set(LED_ON_ALIVE);

	TickType_t last_wake_time = xTaskGetTickCount();

	while(1) {

		can_com_rai_t data;
		memset(&data, 0, sizeof(data));

		uint16_t pwm[SUMD_CHNL_NUM] = {0};

		if (drv_sumd_get_pwm(pwm) == ESP_OK) {
			// We have valid SUMD data

			if (pwm[7] > 1500) {
				// The pilot wants control, lets give him control
				drv_pwm_set_arr(pwm, PWM_CH_NUM);
				drv_led_set(LED_ON_ALIVE);

				ESP_LOGI(__FILE__, "Pilot control");

			} else {
				// The pilot wants the CAN to have control

				if (can_com_rai_get(&data) == ESP_OK) {
					// Since we have valid CAN data lets use that
					drv_pwm_set_arr(data.chnl, PWM_CH_NUM);
					drv_led_set(LED_SLOW);

					ESP_LOGI(__FILE__, "RPI control");

				} else {
					// The pilot wants the CAN to have control, but CAN has
					// no valid data. Lets keep control with the pilot and
					// inidcate the error
					drv_pwm_set_arr(pwm, PWM_CH_NUM);
					drv_led_set(LED_FAST);

					ESP_LOGE(__FILE__, "Pilot control, no CAN data");
				}
			}
		} else {
			// We do not have valid SUMD data

			if (can_com_rai_get(&data) == ESP_OK) {

				// Since we have valid CAN data, lets use that instead
				// Still this is an error
				// TODO check if we should really trust the system here
				// or instead rather switch to internal PID

				drv_pwm_set_arr(data.chnl, PWM_CH_NUM);
				drv_led_set(LED_FAST);

				ESP_LOGE(__FILE__, "RPI control, no PWM data");

			} else {
				// We also dont have valid CAN data

				// TODO this is the internal failsafe
				// We dont get data from the RPI, nor data from the SUMD
				// This is where we need to rely on failsafe

				// Lets turn the motor off and put the plane into a right turn
				pwm[0] = 900;
				pwm[1] = 1100;
				pwm[2] = 1900;
				pwm[3] = 1500;
				pwm[4] = 1900;
				pwm[5] = 1500;
				pwm[6] = 1500;
				pwm[7] = 1500;
				pwm[8] = 1500;

				drv_pwm_set_arr(pwm, PWM_CH_NUM);
				drv_led_set(LED_FAST);

				ESP_LOGE(__FILE__, "FAILSAFE");
			}
		}

		// printf("data.time: %f ", data.time);
		// printf("idata.chnl[0]: %u \n", data.chnl[0]);

		delay_until_ms(&last_wake_time, 10);
	}

	// If we get here, something went badly wrong. Reset the system
	esp_restart();
}
