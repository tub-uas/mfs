#include "drv_led.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "util.h"

#define LED_MSG_PIN 23

// todo: use atomic
led_state_t led_state = LED_OFF;

esp_err_t drv_led_init() {

	ESP_LOGI(__FILE__, "Initalizing LED driver");

	gpio_config_t io_conf;
	io_conf.pin_bit_mask = 1<<LED_MSG_PIN;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	esp_err_t ret = gpio_config(&io_conf);

	if (xTaskCreate(drv_led_worker, "drv_led_worker", 4096, NULL, 5, NULL) < 0) {
		ESP_LOGE(__FILE__, "Cant start LED driver worker");
		return ESP_FAIL;
	}

	ret |= drv_led_set(LED_OFF);

	return ret;
}

esp_err_t drv_led_set(led_state_t state) {
	led_state = state;
	return ESP_OK;
	// return gpio_set_level(LED_MSG_PIN, state);
}

void drv_led_worker() {

	ESP_LOGI(__FILE__, "LED driver worker started");

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {

		uint32_t period = 0;
		float duty_cycle = 0;

		switch (led_state) {
			case LED_OFF:
				duty_cycle = 0.0;
				break;

			case LED_FAST:
				period = 150;
				duty_cycle = 0.5;
				break;

			case LED_MEDIUM:
				period = 500;
				duty_cycle = 0.5;
				break;

			case LED_SLOW:
				period = 1500;
				duty_cycle = 0.5;
				break;

			case LED_ON_ALIVE:
				period = 1500;
				duty_cycle = 0.95;
				break;

			case LED_ON:
				duty_cycle = 1.0;
				break;

			default:
				ESP_LOGI(__FILE__, "LED driver unknown state");
				break;
		}

		static uint32_t gpio_level = 0;
		if (duty_cycle < 0.01) {
			gpio_set_level(LED_MSG_PIN, 0);
			gpio_level = 0;

		} else if (duty_cycle > 0.99) {
			gpio_set_level(LED_MSG_PIN, 1);
			gpio_level = 1;

		} else {

			static uint32_t last_switch_time = 0;
			if (gpio_level == 0) {
				if (get_time_ms() > last_switch_time + period*(1.0-duty_cycle)) {
					gpio_set_level(LED_MSG_PIN, 1);
					gpio_level = 1;
					last_switch_time = get_time_ms();
				}

			} else {
				if (get_time_ms() > last_switch_time + period*duty_cycle) {
					gpio_set_level(LED_MSG_PIN, 0);
					gpio_level = 0;
					last_switch_time = get_time_ms();
				}
			}
		}

		delay_until_ms(&last_wake_time, 20);
	}

	ESP_LOGW(__FILE__, "LED driver worker stopped");
}

void drv_led_test() {

	ESP_LOGI(__FILE__, "LED driver test");

	while(1) {

		drv_led_set(LED_ON);
		delay_ms(1000);
		drv_led_set(LED_OFF);
		delay_ms(1000);
		drv_led_set(LED_FAST);
		delay_ms(5000);
		drv_led_set(LED_SLOW);
		delay_ms(10000);

	}
}
