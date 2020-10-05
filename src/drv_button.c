#include "drv_button.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "util.h"

#define BUTTON_MSG_PIN 34ULL

esp_err_t drv_button_init() {

	ESP_LOGI(__FILE__, "Initalizing Button Driver");

	gpio_config_t io_conf;
	io_conf.pin_bit_mask = 1ULL<<BUTTON_MSG_PIN;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;

	return gpio_config(&io_conf);

}

uint32_t drv_button_pressed() {
	return gpio_get_level(BUTTON_MSG_PIN) ^ 0x01;
}

void drv_button_test() {

	ESP_LOGI(__FILE__, "Button Driver Test");

	while (1) {
		printf("Button Status: %u \n", drv_button_pressed());
		delay_ms(500);
	}
}
