#include "can_com_gps.h"

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

esp_err_t can_com_gps_init() {
	return drv_can_init(NULL, 0);
}

esp_err_t can_com_gps_send(can_com_gps_t data) {

	can_message_t message;
	message.flags = CAN_MSG_FLAG_NONE;
	message.data_length_code = 8;

	message.identifier = CAN_ID_GPS_DATA0;
	memcpy(&message.data[0], &data.time, sizeof(float));
	memcpy(&message.data[4], &data.cog, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_GPS_DATA1;
	memcpy(&message.data[0], &data.second, sizeof(uint8_t));
	memcpy(&message.data[1], &data.minute, sizeof(uint8_t));
	memcpy(&message.data[2], &data.hour, sizeof(uint8_t));
	memcpy(&message.data[3], &data.day, sizeof(uint8_t));
	memcpy(&message.data[4], &data.month, sizeof(uint8_t));
	memcpy(&message.data[5], &data.year, sizeof(uint16_t));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_GPS_DATA2;
	memcpy(&message.data[0], &data.fix, sizeof(uint8_t));
	memcpy(&message.data[1], &data.fix_mode, sizeof(uint8_t));
	memcpy(&message.data[2], &data.sats_in_view, sizeof(uint8_t));
	memcpy(&message.data[3], &data.sats_in_use, sizeof(uint8_t));
	memcpy(&message.data[4], &data.valid, sizeof(uint8_t));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_GPS_DATA3;
	memcpy(&message.data[0], &data.latitude, sizeof(float));
	memcpy(&message.data[4], &data.longitude, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_GPS_DATA4;
	memcpy(&message.data[0], &data.altitude, sizeof(float));
	memcpy(&message.data[4], &data.speed, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_GPS_DATA5;
	memcpy(&message.data[0], &data.dop_h, sizeof(float));
	memcpy(&message.data[4], &data.dop_p, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_GPS_DATA6;
	memcpy(&message.data[0], &data.dop_v, sizeof(float));
	memcpy(&message.data[4], &data.variation, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGW(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	return ESP_OK;

}
