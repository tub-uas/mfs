#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <stdint.h>
#include <math.h>

#include "esp_err.h"

#include "can_com_ahrs.h"

esp_err_t attitude_init();

float adaptive_gain(float err_acc);

void attitude_acc_q(float acc[3], float q_res[4]);

void attitude_acc_mag_q(float acc_q[4], float mag[3], float mag_res_q[4], float acc_mag_q[4]);

void attitude_gyro_q(float gyr[3], float est_q[4], float gyr_q[4], float millis_delta);

void attitude_quat_eul_conv(float q[4], float att[3]);

void attitude_est(float acc[3], float gyr[3], float mag[3], float att[3], float millis_delta);

esp_err_t get_attitude(can_com_ahrs_t *data);

void attitude_worker();

void attitude_test();

#endif // ATTITUDE_ESTIMATION_H
