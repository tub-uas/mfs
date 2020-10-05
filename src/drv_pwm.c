#include "drv_pwm.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "soc/mcpwm_periph.h"
#include "driver/mcpwm.h"

#include "util.h"

#define PWM_PIN_0 5
#define PWM_PIN_1 18
#define PWM_PIN_2 19
#define PWM_PIN_3 27
#define PWM_PIN_4 26
#define PWM_PIN_5 25
#define PWM_PIN_6 33
#define PWM_PIN_7 32
#define PWM_PIN_AUX 2
#define PWM_PIN_UARTRX 16
#define PWM_PIN_UARTTX 17
#define PWM_EN_PIN 4

esp_err_t drv_pwm_init() {

	ESP_LOGI(__FILE__, "Initalizing PWM Driver");

	gpio_config_t io_conf;
	io_conf.pin_bit_mask = 1<<PWM_EN_PIN;
	io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	esp_err_t ret = gpio_config(&io_conf);

	ret |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_PIN_0);
	ret |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_PIN_1);
	ret |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_PIN_2);
	ret |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PWM_PIN_3);
	ret |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_PIN_4);
	ret |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PWM_PIN_5);
	ret |= mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PWM_PIN_6);
	ret |= mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, PWM_PIN_7);

	#ifdef ENABLE_PWM_AUX
	ret |= mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PWM_PIN_AUX);
	#endif

	#ifdef ENABLE_PWM_UARTRX
	ret |= mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, PWM_PIN_UARTRX);
	#endif

	#ifdef ENABLE_PWM_UARTTX
	ret |= mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, PWM_PIN_UARTTX);
	#endif

	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;
	pwm_config.cmpr_a = 0;
	pwm_config.cmpr_b = 0;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	ret |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	ret |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
	ret |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
	ret |= mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
	ret |= mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
	ret |= mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &pwm_config);

	drv_pwm_output(PWM_ENABLE);

	return ret;
}

esp_err_t drv_pwm_set(uint32_t ch, uint16_t us) {

	if (us < PWM_US_MIN || PWM_US_MAX < us) {
		ESP_LOGE(__FILE__, "Invalid PWM us value");
		return ESP_ERR_INVALID_ARG;
	}

	switch (ch) {
		case PWM_CH_0:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, us);

		case PWM_CH_1:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, us);

		case PWM_CH_2:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, us);

		case PWM_CH_3:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, us);

		case PWM_CH_4:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, us);

		case PWM_CH_5:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, us);

		case PWM_CH_6:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, us);

		case PWM_CH_7:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, us);

		#ifdef ENABLE_PWM_AUX
		case PWM_CH_AUX:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, us);
		#endif

		#ifdef ENABLE_PWM_UARTRX
		case PWM_CH_UARTRX:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, us);
		#endif

		#ifdef ENABLE_PWM_UARTTX
		case PWM_CH_UARTTX:
			return mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, us);
		#endif

		default:
			ESP_LOGE(__FILE__, "Invalid PWM Channel");
			return ESP_ERR_INVALID_ARG;
	}

}

esp_err_t drv_pwm_set_arr(uint16_t chnl[], uint32_t len) {

	esp_err_t ret = ESP_OK;
	for (uint32_t i=0; i<len; i++) {
		ret |= drv_pwm_set(i, chnl[i]);
	}

	return ret;
}


esp_err_t drv_pwm_output(pwm_output_t status) {
	return gpio_set_level(PWM_EN_PIN, status);
}

pwm_output_t drv_pwm_get_output() {
	return (gpio_get_level(PWM_EN_PIN) ? PWM_ENABLE : PWM_DISABLE);
}

void drv_pwm_test() {

	ESP_LOGI(__FILE__, "PWM Driver Test");

	while(1) {

		// for(uint32_t i=0; i<100; i+=5) {
		// 	for(uint32_t j=0; j<PWM_CH_NUM; j++) {
		// 		drv_pwm_set(j, 1000+100*j+i);
		// 	}
		// 	delay_ms(50);
		// }

		drv_pwm_set(PWM_CH_0, 800);
		drv_pwm_set(PWM_CH_1, 1000);
		drv_pwm_set(PWM_CH_2, 1200);
		drv_pwm_set(PWM_CH_3, 1400);
		drv_pwm_set(PWM_CH_4, 1600);
		drv_pwm_set(PWM_CH_5, 1800);
		drv_pwm_set(PWM_CH_6, 2000);
		drv_pwm_set(PWM_CH_7, 2200);
		drv_pwm_set(PWM_CH_AUX, 1000);
		// drv_pwm_set(PWM_CH_UARTRX, 1200);
		// drv_pwm_set(PWM_CH_UARTTX, 1400);

		
		delay_ms(50);
	}

}
