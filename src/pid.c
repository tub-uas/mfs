#include "pid.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

esp_err_t pid_init(pid_ctrl_t *pid, float k_p, float k_i, float k_d,
             float out_min, float out_max, float deadband, enum pid_special special) {

	pid->k_p = k_p;
	pid->k_i = k_i;
	pid->k_d = k_d;

	if (out_min >= out_max) {
		ESP_LOGE(__FILE__, "Out_min has to be smaller than out_max");
		return ESP_ERR_INVALID_ARG;
	}

	pid->out_min = out_min;
	pid->out_max = out_max;
	pid->deadband = deadband;
	pid->err_acc = 0.0;
	pid->prev_in = 0.0;

	pid->special = special;

	return ESP_OK;
}

esp_err_t pid_run(pid_ctrl_t *pid, float dt, float set, float in, float *out) {

	if (dt <= 0.0) {
		ESP_LOGE(__FILE__, "Timestep has to be greater than zero");
		return ESP_FAIL;
	}

	// Calculate the control error
	float err = set - in;

	// Check for special exponential function
	if (pid->special == EXPO) {
		if (err > 0.0) {
			err = exp(err) - 1;
		} else {
			err = -exp(-err) + 1;
		}
	}

	// Check, when deadband active, if inside deadband
	if (fabs(err) < pid->deadband && pid->deadband != 0.0) {
		*out = 0.0;
		return 1;
	}

	// Check for special wrap around function
	if (pid->special == WRAP) {
		if (err < pid->out_min) {
			err += 2.0 * pid->out_max;
		} else if (err > pid->out_max) {
			err += 2.0 * pid->out_min;
		}
	}

	// Calculate integral error
	float integ = err * dt + pid->err_acc;

	// Calculate derivative error
	// Do derivate of only process variable to avoid command glitches
	float deriv = -(pid->prev_in - in) / dt;

	// Calculate overall error
	*out = err * pid->k_p + integ * pid->k_i + deriv * pid->k_d;

	// Bound output to set limits, this is also our anti windup logic
	if (*out < pid->out_min) {
		*out = pid->out_min;
	} else if (*out > pid->out_max) {
		*out = pid->out_max;
	} else {
		pid->err_acc += err * dt;
	}

	// Store current error in prev variable for next iteration
	pid->prev_in = in;

	return ESP_OK;
}

esp_err_t pid_reset(pid_ctrl_t *pid) {

	pid->err_acc = 0.0;
	pid->prev_in = 0.0;

	return ESP_OK;
}

esp_err_t pid_update(pid_ctrl_t *pid, float k_p, float k_i, float k_d) {

	pid->k_p = k_p;
	pid->k_i = k_i;
	pid->k_d = k_d;

	return ESP_OK;
}
