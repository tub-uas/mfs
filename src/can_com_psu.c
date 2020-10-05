#include "can_com_psu.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/can.h"

#include "can_ids.h"
#include "can_meta.h"
#include "drv_can.h"
#include "util.h"

esp_err_t can_com_psu_init() {
	return drv_can_init(NULL, 0);
}

esp_err_t can_com_psu_send(can_com_psu_t data) {

	can_message_t message;
	message.flags = CAN_MSG_FLAG_NONE;
	message.data_length_code = 8;

	message.identifier = CAN_ID_PSU_DATA0;
	memcpy(&message.data[0], &data.sense_main_volt, sizeof(float));
	memcpy(&message.data[4], &data.sense_main_curr, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_PSU_DATA1;
	memcpy(&message.data[0], &data.sense_main_pow, sizeof(float));
	memcpy(&message.data[4], &data.sense_pwr_volt, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_PSU_DATA2;
	memcpy(&message.data[0], &data.sense_pwr_curr, sizeof(float));
	memcpy(&message.data[4], &data.sense_pwr_pow, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_PSU_DATA3;
	memcpy(&message.data[0], &data.sense_sys_volt, sizeof(float));
	memcpy(&message.data[4], &data.sense_sys_curr, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_PSU_DATA4;
	memcpy(&message.data[0], &data.sense_sys_pow, sizeof(float));
	memcpy(&message.data[4], &data.sense_time, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	return ESP_OK;

}
