#include "drv_sense.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#include "util.h"

/* We are using ADC1, since ADC2 is not usable when wifi is enabled. */
#define SENSE_VPOWER_CH  ADC1_GPIO35_CHANNEL
#define SENSE_VSYSTEM_CH ADC1_GPIO36_CHANNEL	// aka. SENSOR_VP

/* This is how many times we will take a measurment to get an average value */
#define SENSE_VPOWER_AVG 10
#define SENSE_VSYSTEM_AVG 10

/* This linear correction function helps to correct for internal esp32 adc errors
 * and external voltage divider errors. Use excel to select at least 5 Data Points
 * and do a linear regression with A = SLOPE() * 1024 and B = INTERCEPT().
 * mv = (A / 1024) * raw + B
 *    = (A * raw) / 1024 + B
 */
#define SENSE_ADC_FUNC_A 1431
#define SENSE_ADC_FUNC_B 344

uint32_t drv_sense_get_raw(adc_channel_t ch, uint32_t samples);

float drv_sense_raw_to_mv(uint32_t raw);

esp_err_t drv_sense_init() {

	ESP_LOGI(__FILE__, "Initalizing internal Voltage Sensor Driver");

	esp_err_t ret = adc1_config_width(ADC_WIDTH_BIT_12);

	ret |= adc1_config_channel_atten(SENSE_VPOWER_CH, ADC_ATTEN_DB_6);
	ret |= adc1_config_channel_atten(SENSE_VSYSTEM_CH, ADC_ATTEN_DB_6);

	return ret;
}

float drv_sense_vsystem() {
	return drv_sense_raw_to_mv(drv_sense_get_raw(SENSE_VSYSTEM_CH,
	                                             SENSE_VPOWER_AVG));
}

float drv_sense_vpower() {
	return drv_sense_raw_to_mv(drv_sense_get_raw(SENSE_VPOWER_CH,
	                                             SENSE_VPOWER_AVG));
}

uint32_t drv_sense_get_raw(adc_channel_t ch, uint32_t samples) {
	uint32_t raw = 0;
	for(uint32_t i=0; i<samples; i++) {
		raw += adc1_get_raw(ch);
		delay_ms(1);
	}
	return raw / samples;
}

float drv_sense_raw_to_mv(uint32_t raw) {
	return (((SENSE_ADC_FUNC_A * raw) >> 10) + SENSE_ADC_FUNC_B) / 1000.0;
}

void drv_sense_test() {

	ESP_LOGI(__FILE__, "Internal Voltage Sensor Driver Test");

	while(1) {
		uint32_t raw = drv_sense_get_raw(SENSE_VSYSTEM_CH, 500);
		printf("SENSE_VSYSTEM_CH vol: %f    ", drv_sense_raw_to_mv(raw));
		raw = drv_sense_get_raw(SENSE_VPOWER_CH, 500);
		printf("SENSE_VPOWER_CH vol: %f \n", drv_sense_raw_to_mv(raw));

		// printf("vsystem: %f, vpower: %f \n", drv_sense_vsystem(), drv_sense_vpower());

		delay_ms(1);
	}
}
