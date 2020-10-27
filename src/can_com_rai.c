#include "can_com_rai.h"

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

uint32_t in_ids[] = {CAN_ID_FCC_DATA0,
                     CAN_ID_FCC_DATA1,
                     CAN_ID_FCC_DATA2,
                     CAN_ID_FCC_DATA3};

static can_com_rai_t can_com_rai_data;
static SemaphoreHandle_t can_com_rai_sem = NULL;
static uint32_t can_com_rai_valid_time = 0;

void can_com_rai_worker();

esp_err_t can_com_rai_init() {

	if (drv_can_init(in_ids, CAN_META_RAI_MSG_NUM) != ESP_OK) {
		return ESP_FAIL;
	}

	can_com_rai_sem = xSemaphoreCreateMutex();
	if (can_com_rai_sem == NULL) {
		ESP_LOGE(__FILE__, "Cant create CAN Com Rai mutex");
		return ESP_FAIL;
	}

	if (xTaskCreate(can_com_rai_worker, "can_com_rai_worker", 4096, NULL, 15, NULL) < 0) {
		ESP_LOGE(__FILE__, "Cant start CAN Com Rai worker");
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t can_com_rai_send(can_com_rai_t data) {

	can_message_t message;
	message.flags = CAN_MSG_FLAG_NONE;
	message.data_length_code = 8;

	message.identifier = CAN_ID_RAI_DATA0;
	memcpy(&message.data[0], &data.chnl[0], message.data_length_code);
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGE(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_RAI_DATA1;
	memcpy(&message.data[0], &data.chnl[4], message.data_length_code);
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGE(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_RAI_DATA2;
	memcpy(&message.data[0], &data.chnl[8], message.data_length_code);
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGE(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	message.identifier = CAN_ID_RAI_DATA3;
	message.data_length_code = 4;
	memcpy(&message.data[0], &data.time, sizeof(float));
	if (can_transmit(&message, pdMS_TO_TICKS(CAN_TIMEOUT)) != ESP_OK) {
		ESP_LOGE(__FILE__, "Failed to queue CAN message for transmission, id: 0x%x",
		         message.identifier);
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t can_com_rai_get(can_com_rai_t *data) {
	if (xSemaphoreTake(can_com_rai_sem, 10 / portTICK_PERIOD_MS) == true) {
		if (get_time_ms() > can_com_rai_valid_time + 500) {
			xSemaphoreGive(can_com_rai_sem);
			return ESP_FAIL;
		} else {
			memcpy(data, &can_com_rai_data, sizeof(can_com_rai_t));
			xSemaphoreGive(can_com_rai_sem);
			return ESP_OK;
		}
	} else {
		return ESP_ERR_TIMEOUT;
	}
}

void can_com_rai_worker() {

	ESP_LOGI(__FILE__, "CAN Com Rai Worker Started");

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {

		can_com_rai_t data;
		if (can_com_rai_recv(&data) == ESP_OK) {
			if (xSemaphoreTake(can_com_rai_sem, 10 / portTICK_PERIOD_MS) == true) {
				memcpy(&can_com_rai_data, &data, sizeof(can_com_rai_t));
				can_com_rai_valid_time = get_time_ms();
				xSemaphoreGive(can_com_rai_sem);
			} else {
				ESP_LOGE(__FILE__, "CAN Com Rai mutex error");
			}
		} else {
			ESP_LOGE(__FILE__, "CAN Com Rai worker no new data");
		}

		delay_until_ms(&last_wake_time, 5);
	}
}

esp_err_t can_com_rai_recv(can_com_rai_t *data) {

	for (int i=0; i<CAN_META_RAI_MSG_NUM; i++) {

		can_message_t message;

		if (can_receive(&message, pdMS_TO_TICKS(50)) != ESP_OK) {
			return ESP_FAIL;

		} else {

			if (message.identifier != in_ids[i]) {
				ESP_LOGE(__FILE__, "Recv: CAN id should be %x but is %x", in_ids[i], message.identifier);
				return ESP_FAIL;
			}

			if (i == (CAN_META_RAI_MSG_NUM-1)) {
				memcpy(&data->time, &message.data[0], sizeof(float));
				break;
			}

			for (int j=0; j<CAN_META_RAI_CHNL_PER_MSG; j++) {

				uint16_t pwm = message.data[2*j+1] << 8 | message.data[2*j];
				int idx = CAN_META_RAI_CHNL_PER_MSG*(message.identifier-CAN_ID_FCC_DATA0)+j;

				if (pwm > 2200 || pwm < 800) {
					ESP_LOGE(__FILE__, "Recv: PWM out of bounds %d at channel %d", pwm, idx);
					return ESP_FAIL;
				}

				data->chnl[idx] = pwm;
			}
		}
	}

	return ESP_OK;
}
