#ifndef DRV_PWM_H
#define DRV_PWM_H

#include <stdint.h>

#include "esp_err.h"

#define PWM_US_MIN 500
#define PWM_US_MAX 2500

// Without any extra channels this value is 8
#define PWM_CH_NUM 9

// For every channel added here, increase the PWM_CH_NUM by one, up to 11
#define ENABLE_PWM_AUX
// #define ENABLE_PWM_UARTRX
// #define ENABLE_PWM_UARTTX


typedef enum {
	PWM_DISABLE = 0,
	PWM_ENABLE
} pwm_output_t;

typedef enum {
	PWM_CH_0 = 0,
	PWM_CH_1,
	PWM_CH_2,
	PWM_CH_3,
	PWM_CH_4,
	PWM_CH_5,
	PWM_CH_6,
	PWM_CH_7,
	PWM_CH_AUX,
	PWM_CH_UARTRX,
	PWM_CH_UARTTX
} pwm_ch_t;


esp_err_t drv_pwm_init();

esp_err_t drv_pwm_set(uint32_t ch, uint16_t us);

esp_err_t drv_pwm_set_arr(uint16_t chnl[], uint32_t len);

esp_err_t drv_pwm_output(pwm_output_t status);

pwm_output_t drv_pwm_get_output();

void drv_pwm_test();

#endif // DRV_PWM_H
