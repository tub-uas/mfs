#include "drv_sumd.h"

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
#include "drv_pwm.h"

// http://www.gh-lounge.de/mediawiki/index.php/Erkl%C3%A4

#define UART_NUM UART_NUM_0
#define UART_BUFFER_SIZE 1024
#define UART_PATTERN_BUFFER_SIZE 16
#define UART_TXD_PIN 17
#define UART_RXD_PIN 16
#define UART_EN_PIN 4

#define SUMD_PACKET_LENGTH (5 + 2*SUMD_CHNL_NUM)
#define SUMD_PACKET_START 168
#define SUMD_PACKET_TX_ON 1
#define SUMD_PACKET_TX_OFF 129

static SemaphoreHandle_t sumd_decoder_sem = NULL;
static QueueHandle_t uart_event_queue = NULL;
static uint16_t servo_pwm[SUMD_CHNL_NUM] = {0};
static uint32_t servo_pwm_valid_time = 0;

esp_err_t drv_sumd_decode();

void drv_sumd_worker();

esp_err_t drv_sumd_init() {

	ESP_LOGI(__FILE__, "Initalizing SUMD Driver");

	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};

	esp_err_t ret = uart_driver_install(UART_NUM, 2*UART_BUFFER_SIZE,
	                                    0, UART_PATTERN_BUFFER_SIZE,
	                                    &uart_event_queue, 0);

	ret |= uart_param_config(UART_NUM, &uart_config);

	ret |= uart_set_pin(UART_NUM, UART_TXD_PIN, UART_RXD_PIN,
	                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	/* no pattern detection needed for now */
	// uart_enable_pattern_det_intr(UART_NUM, (char)168, 1, 9, 0, 0);
	// uart_pattern_queue_reset(UART_NUM, UART_PATTERN_BUFFER_SIZE);

	ret |= uart_flush(UART_NUM);

	sumd_decoder_sem = xSemaphoreCreateMutex();
	if (sumd_decoder_sem == NULL) {
		ESP_LOGE(__FILE__, "Cant create SUMD mutex");
		return ESP_FAIL;
	}

	if (xTaskCreate(drv_sumd_worker, "drv_sumd_worker", 4096, NULL, 5, NULL) < 0) {
		ESP_LOGE(__FILE__, "Cant start SUMD driver worker");
		return ESP_FAIL;
	}

	if (drv_pwm_get_output() != PWM_ENABLE) {
		ESP_LOGE(__FILE__, "SUMD input not enabled");
		return ESP_FAIL;
	}

	return ret;
}

esp_err_t drv_sumd_get_pwm(uint16_t pwm_dest[]) {
	if (xSemaphoreTake(sumd_decoder_sem, 10 / portTICK_PERIOD_MS) == true) {
		if (get_time_ms() > servo_pwm_valid_time + 500) {
			// ESP_LOGE(__FILE__, "SUMD driver could not get valid pwm data");
			xSemaphoreGive(sumd_decoder_sem);
			return ESP_FAIL;
		} else {
			memcpy(pwm_dest, servo_pwm, sizeof(uint16_t) * SUMD_CHNL_NUM);
			xSemaphoreGive(sumd_decoder_sem);
			return ESP_OK;
		}
	} else {
		return ESP_ERR_TIMEOUT;
	}
}

esp_err_t drv_sumd_decode() {

	uint32_t len = 0;
	uart_get_buffered_data_len(UART_NUM, &len);

	if (len < SUMD_PACKET_LENGTH) {
		ESP_LOGW(__FILE__,
			"SUMD packet too short, is %d should %d, check SUMD_CHNL_NUM",
			len, SUMD_PACKET_LENGTH);
		return ESP_FAIL;
	}

	uint8_t buffer[SUMD_PACKET_LENGTH];
	uart_read_bytes(UART_NUM, buffer, SUMD_PACKET_LENGTH, 100 / portTICK_PERIOD_MS);

	if ((buffer[0] != SUMD_PACKET_START) ||
	    (buffer[1] != SUMD_PACKET_TX_ON && buffer[1] != SUMD_PACKET_TX_OFF) ||
	    (buffer[2] != SUMD_CHNL_NUM)) {
		ESP_LOGW(__FILE__, "SUMD packet header error");
		return ESP_FAIL;
	}

	uint16_t crc  = buffer[SUMD_PACKET_LENGTH-2] << 8;
	         crc |= buffer[SUMD_PACKET_LENGTH-1];

	/* internal crc algo does not seem to work */
	// uint16_t crc_cal = crc16_le(0, buffer, SUMD_PACKET_LENGTH-2);
	// uint16_t crc_cal = crc16_be(0, buffer, SUMD_PACKET_LENGTH-2);

	uint16_t crc_cal = 0;
	for (uint32_t i=0; i<SUMD_PACKET_LENGTH-2; i++) {
		crc_cal ^= (uint16_t) (buffer[i] << 8);
		for (uint32_t j = 0; j < 8; ++j) {
			if ((crc_cal & 0x8000) != 0) {
				crc_cal = (uint16_t) (crc_cal << 1)^0x1021;
			} else {
				crc_cal <<= 1;
			}
		}
	}

	if (crc != crc_cal) {
		ESP_LOGE(__FILE__, "SUMD packet crc error");
		return ESP_ERR_INVALID_CRC;
	}

	uint16_t servo_pwm_tmp[SUMD_CHNL_NUM] = {0};
	for (uint32_t i=0; i<SUMD_CHNL_NUM; i++) {
		servo_pwm_tmp[i]  = (uint16_t) buffer[i*2+3] << 8;
		servo_pwm_tmp[i] |= (uint16_t) buffer[i*2+4];
		servo_pwm_tmp[i] >>= 3;
		if (servo_pwm_tmp[i] < 800 || 2000 < servo_pwm_tmp[i]) {
			ESP_LOGE(__FILE__, "SUMD packet pwm error");
			return ESP_ERR_INVALID_SIZE;
		}
	}

	if (xSemaphoreTake(sumd_decoder_sem, 10 / portTICK_PERIOD_MS) == true) {
		memcpy(servo_pwm, servo_pwm_tmp, sizeof(uint16_t) * SUMD_CHNL_NUM);
		servo_pwm_valid_time = get_time_ms();
		xSemaphoreGive(sumd_decoder_sem);
	} else {
		ESP_LOGE(__FILE__, "SUMD packet mutex error");
		return ESP_ERR_TIMEOUT;
	}

	return ESP_OK;
}

void drv_sumd_worker() {

	ESP_LOGI(__FILE__, "SUMD Driver Worker Started");

	while (1) {

		uart_event_t event;
		if (xQueueReceive(uart_event_queue, (void*)&event, 100/portTICK_PERIOD_MS)) {

			switch (event.type) {
				case UART_DATA:
					if (drv_sumd_decode() != ESP_OK) {
						ESP_LOGE(__FILE__, "SUMD packet decode error");
						uart_flush(UART_NUM);
						xQueueReset(uart_event_queue);
					} else {
						// ESP_LOGI(__FILE__, "SUMD packet decode success");
					}
					break;

				case UART_FIFO_OVF:
					ESP_LOGE(__FILE__, "SUMD UART hardware FIFO overflow");
					uart_flush(UART_NUM);
					xQueueReset(uart_event_queue);
					break;

				case UART_BUFFER_FULL:
					ESP_LOGE(__FILE__, "SUMD UART ring buffer full");
					uart_flush(UART_NUM);
					xQueueReset(uart_event_queue);
					break;

				case UART_BREAK:
					ESP_LOGE(__FILE__, "SUMD UART Rx break");
					break;

				case UART_PARITY_ERR:
					ESP_LOGE(__FILE__, "SUMD UART parity error");
					break;

				case UART_FRAME_ERR:
					ESP_LOGE(__FILE__, "SUMD UART frame error");
					break;

				case UART_PATTERN_DET:
					ESP_LOGW(__FILE__, "SUMD UART pattern detected");
					break;

				default:
					ESP_LOGW(__FILE__, "SUMD UART unknown uart event type: %d", event.type);
					break;
			}

		} else {
			ESP_LOGE(__FILE__, "SUMD driver worker timed out");
		}
	}

	ESP_LOGW(__FILE__, "SUMD driver worker stopped");
}

void drv_sumd_test() {

	ESP_LOGI(__FILE__, "SUMD driver test");

	while (1) {
		uint16_t pwm[SUMD_CHNL_NUM] = {0};
		if (drv_sumd_get_pwm(pwm) == ESP_OK) {
			for (uint32_t i=0; i<SUMD_CHNL_NUM; i++) {
				printf("%u:%d  ", i, pwm[i]);
			}
			printf("\n");
		} else {
			printf("Cant get PWM from driver \n");
		}
		delay_ms(20);
	}
}
