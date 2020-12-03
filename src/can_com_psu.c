#include "can_com_psu.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/can.h"

#include "can_ids.h"
#include "can_meta.h"
#include "drv_can.h"
#include "util.h"

static uint32_t in_ids[] = {CAN_ID_PSU_DATA0,
                            CAN_ID_PSU_DATA1,
                            CAN_ID_PSU_DATA2,
                            CAN_ID_PSU_DATA3,
                            CAN_ID_PSU_DATA4};

static can_com_psu_t can_com_psu_data;
static SemaphoreHandle_t can_com_psu_sem = NULL;
static uint32_t can_com_psu_valid_time = 0;

void can_com_psu_worker();

esp_err_t can_com_psu_init(uint8_t receive) {

	/* Only interested in transmitting data */
	if (!receive) {
		return drv_can_init(NULL, 0);
	}

	if (drv_can_init(in_ids, CAN_META_PSU_MSG_NUM) != ESP_OK) {
		return ESP_FAIL;
	}

	can_com_psu_sem = xSemaphoreCreateMutex();
	if (can_com_psu_sem == NULL) {
		ESP_LOGE(__FILE__, "Cant create CAN Com Psu mutex");
		return ESP_FAIL;
	}

	if (xTaskCreate(can_com_psu_worker, "can_com_psu_worker", 4096, NULL, 15, NULL) < 0) {
		ESP_LOGE(__FILE__, "Cant start CAN Com Psu worker");
		return ESP_FAIL;
	}

	return ESP_OK;
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

esp_err_t can_com_psu_get(can_com_psu_t *data) {
	if (xSemaphoreTake(can_com_psu_sem, 10 / portTICK_PERIOD_MS) == true) {
		if (get_time_ms() > can_com_psu_valid_time + 500) {
			xSemaphoreGive(can_com_psu_sem);
			return ESP_FAIL;
		} else {
			memcpy(data, &can_com_psu_data, sizeof(can_com_psu_t));
			xSemaphoreGive(can_com_psu_sem);
			return ESP_OK;
		}
	} else {
		return ESP_ERR_TIMEOUT;
	}
}

void can_com_psu_worker() {

	ESP_LOGI(__FILE__, "CAN Com Psu Worker Started");

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {

		can_com_psu_t data;
		if (can_com_psu_recv(&data) == ESP_OK) {
			if (xSemaphoreTake(can_com_psu_sem, 10 / portTICK_PERIOD_MS) == true) {
				memcpy(&can_com_psu_data, &data, sizeof(can_com_psu_t));
				can_com_psu_valid_time = get_time_ms();
				xSemaphoreGive(can_com_psu_sem);
			} else {
				ESP_LOGE(__FILE__, "CAN Com Psu mutex error");
			}
		} else {
			ESP_LOGE(__FILE__, "CAN Com Psu worker no new data");
		}

		delay_until_ms(&last_wake_time, 5);
	}
}

esp_err_t can_com_psu_recv(can_com_psu_t *data) {

	for (int i=0; i<CAN_META_PSU_MSG_NUM; i++) {

		can_message_t message;

		if (can_receive(&message, pdMS_TO_TICKS(50)) != ESP_OK) {
			return ESP_FAIL;

		} else {

			if (message.identifier != in_ids[i]) {
				ESP_LOGE(__FILE__, "Recv: CAN id should be %x but is %x", in_ids[i], message.identifier);
				return ESP_FAIL;
			}

			switch (message.identifier) {
				case CAN_ID_PSU_DATA0:
					memcpy(&data->sense_main_volt, &message.data[0], sizeof(float));
					memcpy(&data->sense_main_curr, &message.data[4], sizeof(float));
					break;

				case CAN_ID_PSU_DATA1:
					memcpy(&data->sense_main_pow, &message.data[0], sizeof(float));
					memcpy(&data->sense_pwr_volt, &message.data[4], sizeof(float));
					break;

				case CAN_ID_PSU_DATA2:
					memcpy(&data->sense_pwr_curr, &message.data[0], sizeof(float));
					memcpy(&data->sense_pwr_pow, &message.data[4], sizeof(float));
					break;

				case CAN_ID_PSU_DATA3:
					memcpy(&data->sense_sys_volt, &message.data[0], sizeof(float));
					memcpy(&data->sense_sys_curr, &message.data[4], sizeof(float));
					break;

				case CAN_ID_PSU_DATA4:
					memcpy(&data->sense_sys_pow, &message.data[0], sizeof(float));
					memcpy(&data->sense_time, &message.data[4], sizeof(float));
					break;

				default:
					ESP_LOGE(__FILE__, "Recv: Unkown CAN id to decode %x \n", message.identifier);
					return ESP_FAIL;
			}
		}
	}

	return ESP_OK;
}
