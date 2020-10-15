#include "drv_nema.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"
#include "esp32/rom/crc.h"

#include "util.h"

#define UART_NUM UART_NUM_1
#define UART_BUFFER_SIZE 1024
#define UART_PATTERN_BUFFER_SIZE 16
#define UART_TXD_PIN 27
#define UART_RXD_PIN 26
#define UART_EN_PIN 4

static QueueHandle_t uart_event_queue = NULL;

esp_err_t drv_nema_init() {

	ESP_LOGI(__FILE__, "Initalizing NEMA Driver");


	uart_config_t uart_config = {
		.baud_rate  = 9600,
		.data_bits  = UART_DATA_8_BITS,
		.parity     = UART_PARITY_DISABLE,
		.stop_bits  = UART_STOP_BITS_1,
		.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
	};

	esp_err_t ret = uart_driver_install(UART_NUM, 2*UART_BUFFER_SIZE,
	                                    0, UART_PATTERN_BUFFER_SIZE,
	                                    &uart_event_queue, 0);

	ret |= uart_param_config(UART_NUM, &uart_config);

	ret |= uart_set_pin(UART_NUM, UART_TXD_PIN, UART_RXD_PIN,
	                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	ret |= uart_flush(UART_NUM);

	return ret;
}

void drv_nema_test() {

	ESP_LOGI(__FILE__, "NEMA driver test");

	while (1) {

		uart_event_t event;

		if (xQueueReceive(uart_event_queue, (void*)&event, 1000/portTICK_PERIOD_MS)) {

			switch (event.type) {

				case UART_DATA: {
					uint8_t buffer[100];
					uart_read_bytes(UART_NUM, buffer, 100, 100 / portTICK_PERIOD_MS);
					printf("LINE: %s", (char*) buffer);
					break;
				}

				case UART_FIFO_OVF:
					ESP_LOGE(__FILE__, "NEMA UART hardware FIFO overflow");
					uart_flush(UART_NUM);
					xQueueReset(uart_event_queue);
					break;

				case UART_BUFFER_FULL:
					ESP_LOGE(__FILE__, "NEMA UART ring buffer full");
					uart_flush(UART_NUM);
					xQueueReset(uart_event_queue);
					break;

				case UART_BREAK:
					ESP_LOGE(__FILE__, "NEMA UART Rx break");
					break;

				case UART_PARITY_ERR:
					ESP_LOGE(__FILE__, "NEMA UART parity error");
					break;

				case UART_FRAME_ERR:
					ESP_LOGE(__FILE__, "NEMA UART frame error");
					break;

				case UART_PATTERN_DET:
					ESP_LOGW(__FILE__, "NEMA UART pattern detected");
					break;

				default:
					ESP_LOGW(__FILE__, "NEMA UART unknown uart event type: %d", event.type);
					break;
			}

		} else {
			ESP_LOGE(__FILE__, "NEMA driver worker timed out");
		}
	}
}
