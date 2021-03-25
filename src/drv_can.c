#include "drv_can.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "util.h"

#define CAN_TX_PIN 21
#define CAN_RX_PIN 22

esp_err_t drv_can_init(uint32_t ids[], uint32_t ids_len) {

	ESP_LOGI(__FILE__, "Initalizing CAN Driver");

	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN,
	                                                           CAN_RX_PIN,
	                                                           TWAI_MODE_NORMAL);

	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	if (ids_len > 0) {

		f_config.acceptance_mask = 0x000;
		f_config.acceptance_code = 0x7FF;

		for (int i=0; i<ids_len; i++) {
			f_config.acceptance_mask |= ids[i];
			f_config.acceptance_code &= ids[i];
		}

		f_config.acceptance_mask ^= f_config.acceptance_code;
		f_config.acceptance_mask &= 0x7FF;
		f_config.acceptance_mask <<= 21;
		f_config.acceptance_mask |= 0x1FFFFF;
		f_config.acceptance_code <<= 21;
	}

	esp_err_t ret_install = twai_driver_install(&g_config, &t_config, &f_config);
	if (ret_install != ESP_OK)
		ESP_LOGE(__FILE__, "Cant install CAN Driver");

	esp_err_t ret_start = twai_start();
	if (ret_start != ESP_OK)
		ESP_LOGE(__FILE__, "Cant start CAN Driver");

	return ret_install | ret_start;
}

void drv_can_test() {

	ESP_LOGI(__FILE__, "CAN Driver Test");

	while (1) {

		twai_message_t message;
		message.flags = TWAI_MSG_FLAG_NONE;
		message.identifier = 0x0A0;
		message.data_length_code = 8;
		for (uint32_t i = 0; i < 8; i++) {
			message.data[i] = i+0xA0;
		}

		if (twai_transmit(&message, pdMS_TO_TICKS(500)) != ESP_OK)
			ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission");

		if (twai_receive(&message, pdMS_TO_TICKS(500)) != ESP_OK) {
			ESP_LOGW(__FILE__, "No CAN message received");
		} else {
			printf("Received CAN Message, id: %u, dlc: %d, data: %x %x %x %x %x %x %x %x \n",
			        message.identifier, message.data_length_code,
			        message.data[0], message.data[1], message.data[2], message.data[3],
			        message.data[4], message.data[5], message.data[6], message.data[7]);
			delay_ms(100);
		}
	}

}
