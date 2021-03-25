#include "can_com_ahrs.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "can_ids.h"
#include "can_meta.h"
#include "drv_can.h"
#include "util.h"

esp_err_t can_com_ahrs_init() {
	return drv_can_init(NULL, 0);
}

esp_err_t can_com_ahrs_send(can_com_ahrs_t data) {

	twai_message_t message;
	message.flags = TWAI_MSG_FLAG_NONE;
	message.data_length_code = 8;

	message.identifier = CAN_ID_AHRS_DATA0;
	memcpy(&message.data[0], &data.time, sizeof(float));
	memcpy(&message.data[4], &data.acc[0], sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_AHRS_DATA1;
	memcpy(&message.data[0], &data.acc[1], sizeof(float));
	memcpy(&message.data[4], &data.acc[2], sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_AHRS_DATA2;
	memcpy(&message.data[0], &data.gyr[0], sizeof(float));
	memcpy(&message.data[4], &data.gyr[1], sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_AHRS_DATA3;
	memcpy(&message.data[0], &data.gyr[2], sizeof(float));
	memcpy(&message.data[4], &data.mag[0], sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_AHRS_DATA4;
	memcpy(&message.data[0], &data.mag[1], sizeof(float));
	memcpy(&message.data[4], &data.mag[2], sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_AHRS_DATA5;
	memcpy(&message.data[0], &data.att[0], sizeof(float));
	memcpy(&message.data[4], &data.att[1], sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_AHRS_DATA6;
	memcpy(&message.data[0], &data.att[2], sizeof(float));
	memcpy(&message.data[4], &data.temp, sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_AHRS_DATA7;
	message.data_length_code = 4;
	memcpy(&message.data[0], &data.press, sizeof(float));
	if (twai_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	return ESP_OK;

}
