#ifndef PID_H
#define PID_H

#include <stdint.h>

#include "esp_err.h"

enum pid_special {
	NORM = 0, // Normal PID controller
	WRAP = 1, // Use wrap around logic
	EXPO = 2 // Use an exponential error function
};

typedef struct {
	float k_p;            // P Gain
	float k_i;            // I Gain
	float k_d;            // D Gain
	float out_min;        // Min and
	float out_max;        // Max output bounds
	float deadband;       // Deadband to allow for response in finite time
	float err_acc;        // Error accumulator, for integral
	float prev_in;        // Previous error, for derivative
	enum pid_special special;  // Special function flag
} pid_ctrl_t;

esp_err_t pid_init(pid_ctrl_t *pid, float k_p, float k_i, float k_d,
             float out_min, float out_max, float deadband, enum pid_special special);

esp_err_t pid_run(pid_ctrl_t *pid, float dt, float set, float in, float *out);

esp_err_t pid_reset(pid_ctrl_t *pid);

esp_err_t pid_update(pid_ctrl_t *pid, float k_p, float k_i, float k_d);

#endif // PID_H
