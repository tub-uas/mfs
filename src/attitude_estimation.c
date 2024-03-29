#include "attitude_estimation.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "string.h"

#include "board.h"
#include "drv_ak8963.h"
#include "drv_ak8963_const.h"
#include "drv_bmp280.h"
#include "drv_mpu9250.h"

// == WITH GPS ==
#include "drv_hmc5883.h"

// == ATTITUDE AND HELPER == 
#include "algo_attitude64.h"
#include "util_math64.h"
#include "util_quaternion64.h"


#include "drv_led.h"
#include "can_com_psu.h"
#include "util_vecmath.h"
#include "util_quaternion.h"
#include "util.h"

#define ADAPTGYROGAIN   0.99
#define ADAPTLOWBOUND   0.1
#define ADAPTUPPBOUND   0.2

#define SQRT2           1.41421356237f
#define INVSQRT2        0.70771067811f

#define K_ACC           4.2735e-2f
#define K_MAG           1.4085e-2f
#define K_BIAS_ACC      6.6774e-7f
#define K_BIAS_MAG      7.0423e-8f

static can_com_ahrs_t ahrs_data;
static SemaphoreHandle_t ahrs_sem = NULL;

esp_err_t attitude_init() {

	drv_led_set(LED_MEDIUM);

	ahrs_sem = xSemaphoreCreateMutex();
	if (ahrs_sem == NULL) {
		ESP_LOGE(__FILE__, "Cant create AHRS mutex");
		return ESP_FAIL;
	}

	if (xTaskCreate(attitude_worker, "attitude_worker", 4096, NULL, 10, NULL) < 0) {
		ESP_LOGE(__FILE__, "Cant start Attitude Worker");
		return ESP_FAIL;
	}

	return ESP_OK;
}

float adaptive_gain(float err_acc) {

	if(err_acc < ADAPTLOWBOUND) {
		return 1.0 * (1-ADAPTGYROGAIN);
	} else if(err_acc > ADAPTUPPBOUND) {
		return 0.0;
	} else {
		return (ADAPTUPPBOUND - err_acc)/(ADAPTLOWBOUND) * (1.0-ADAPTGYROGAIN);
	}
}

void attitude_acc_q(float acc[3], float q_res[4]) {

	if(acc[2] < 0) {
		float sqrtacc = sqrt(1-acc[2]);

		q_res[0] = -acc[1]/(SQRT2*sqrtacc);
		q_res[1] = INVSQRT2*sqrtacc;
		q_res[2] = 0.0;
		q_res[3] = acc[0]/(SQRT2*sqrtacc);

	} else {
		float sqrtacc = sqrt(acc[2]+1);

		q_res[0] = INVSQRT2*sqrtacc;
		q_res[1] = -acc[1]/(SQRT2*sqrtacc);
		q_res[2] = acc[0]/(SQRT2*sqrtacc);
		q_res[3] = 0;
	}

	quaternion_norm(q_res, q_res);
}

void attitude_acc_mag_q(float acc_q[4],float mag[3], float mag_res_q[4], float acc_mag_q[4]) {

	float q_accInv[4] = {0.0};
	quaternion_inverse(acc_q, q_accInv);

	float q_mag[4] = {0, mag[0], mag[1], mag[2]};
	quaternion_prod(q_mag, acc_q, q_mag);

	quaternion_prod(q_accInv, q_mag, q_mag);

	mag[0] = q_mag[1];
	mag[1] = q_mag[2];
	mag[2] = q_mag[3];

	float gamma = mag[0]*mag[0] + mag[1]*mag[1];
	float sqr_gamma = sqrt(gamma);

	if(mag[0] < 0) {
		mag_res_q[0] = mag[1]/sqrt(2*sqr_gamma*(sqr_gamma-mag[0]));
		mag_res_q[1] = 0.0;
		mag_res_q[2] = 0.0;
		mag_res_q[3] = sqrt((sqr_gamma-mag[0])/(2*sqr_gamma));
	} else {
		mag_res_q[0] = sqrt((mag[0]+sqr_gamma) /(2*sqr_gamma));
		mag_res_q[1] = 0.0;
		mag_res_q[2] = 0.0;
		mag_res_q[3] = mag[1] / sqrt(2*sqr_gamma*(mag[0]+sqr_gamma));
	}

	quaternion_norm(mag_res_q, mag_res_q);
	quaternion_prod(acc_q, mag_res_q, acc_mag_q);
	quaternion_norm(acc_mag_q, acc_mag_q);
}

void attitude_gyro_q(float gyr[3], float est_q[4],float gyr_q[4], float millis_delta) {

	float angular = millis_delta*sqrt(gyr[0]*gyr[0] + gyr[1]*gyr[1] + gyr[2]*gyr[2]);

	static float dq[4] = {0.0};
	dq[0] = cos(angular/2);

	float s = (-1)*millis_delta*sin(angular/2)/angular;
	dq[1] = gyr[0] * s;
	dq[2] = gyr[1] * s;
	dq[3] = gyr[2] * s;

	quaternion_norm(dq, dq);
	quaternion_prod(dq, est_q, gyr_q);
	quaternion_norm(gyr_q, gyr_q);
}

void attitude_quat_eul_conv(float Q[4],float att[3]) {

	float arg = (-2.0) * (Q[0]*Q[2]+Q[1]*Q[3]);
	arg = fmin(fmax(arg,-1.0),1.0);

	att[0] = atan2( 2*(Q[2]*Q[3]-Q[0]*Q[1]), 1-2*(Q[1]*Q[1]+Q[2]*Q[2]) );
	att[1] = asin(arg);
	att[2] = atan2( 2*(Q[1]*Q[2]-Q[0]*Q[3]), 1-2*(Q[2]*Q[2]+Q[3]*Q[3]) ) + 0.06370;
}

void attitude_est(float acc[3], float gyr[3], float mag[3], float att[3], float millis_delta) {

	float accn[3] = {0.0};
	float err_acc = fabs(vecmath_norm_3d(acc, accn)-1.0);

	float magn[3] = {0.0};
	vecmath_norm_3d(mag, magn);

	static float acc_q[4] = {0.0};
	attitude_acc_q(accn, acc_q);

	static float mag_q[4] = {0.0};
	static float acc_mag_q[4] = {0.0};
	attitude_acc_mag_q(acc_q, magn, mag_q, acc_mag_q);

	static float gyr_q[4] = {0.0};
	static float est_q[4] = {1.0,0.0,0.0,0.0};
	attitude_gyro_q(gyr, est_q, gyr_q, millis_delta);

	float dotprod = acc_mag_q[0]*gyr_q[0] + acc_mag_q[1]*gyr_q[1] + acc_mag_q[2]*gyr_q[2] + acc_mag_q[3]*gyr_q[3];
	if (dotprod < 0.0) {
		acc_mag_q[0] *= -1.0;
		acc_mag_q[1] *= -1.0;
		acc_mag_q[2] *= -1.0;
		acc_mag_q[3] *= -1.0;
	}

	float gain = adaptive_gain(err_acc);
	for (uint8_t i = 0; i < 4; ++i) {
		est_q[i] = gain * acc_mag_q[i] + (1-gain) * gyr_q[i];
	}

	attitude_quat_eul_conv(est_q, att);
}

esp_err_t get_attitude(can_com_ahrs_t *data) {

	if (xSemaphoreTake(ahrs_sem, 10 / portTICK_PERIOD_MS) == true) {
		memcpy(data, &ahrs_data, sizeof(can_com_ahrs_t));
		xSemaphoreGive(ahrs_sem);
		return ESP_OK;
	} else {
		return ESP_ERR_TIMEOUT;
	}
}

esp_err_t compensate_mag_with_current(float current, ak8963_mag_data_t *mag) {
	mag->x = mag->x - MAG_CS_1 * current;
	mag->y = mag->y - MAG_CS_2 * current;
	mag->z = mag->z - MAG_CS_3 * current;

	return ESP_OK;
}

void attitude_worker() {

	ESP_LOGI(__FILE__, "Starting Attitude Worker");

	TickType_t last_wake_time = xTaskGetTickCount();

	drv_led_set(LED_ON_ALIVE);
	double q_att[4] = {1.0f,0.0f,0.0f,0.0f};
	double b_gyr[3] = {0.0f,0.0f,0.0f};
	while (1) {

		can_com_psu_t psu_data;
		memset(&psu_data, 0, sizeof(psu_data));
		if (can_com_psu_get(&psu_data) != ESP_OK) {
			ESP_LOGE(__FILE__, "Could not get PSU data from CAN");
		}
		// printf("Main %5.2f %5.2f %5.2f   ", psu_data.sense_main_volt, psu_data.sense_main_curr, psu_data.sense_main_pow);
		// printf("Pwr  %5.2f %5.2f %5.2f   ", psu_data.sense_pwr_volt, psu_data.sense_pwr_curr, psu_data.sense_pwr_pow);
		// printf("sys  %5.2f %5.2f %5.2f \n", psu_data.sense_sys_volt, psu_data.sense_sys_curr, psu_data.sense_sys_pow);

		mpu9250_acc_data_t acc;
		memset(&acc, 0, sizeof(acc));
		if (drv_mpu9250_read_acc(&acc) != ESP_OK) {
			ESP_LOGE(__FILE__, "Could not read accel data from mpu9250");
		}
		drv_mpu9250_correct_acc_data(&acc);
		

		mpu9250_gyr_data_t gyr;
		memset(&gyr, 0, sizeof(gyr));
		if (drv_mpu9250_read_gyr(&gyr) != ESP_OK) {
			ESP_LOGE(__FILE__, "Could not read gyro data from mpu9250");
		}
		drv_mpu9250_correct_gyr_data(&gyr);

		ak8963_mag_data_t mag;
		memset(&mag, 0, sizeof(mag));
		static ak8963_mag_data_t last_mag;
		esp_err_t ret = drv_ak8963_read_mag(&mag);
		if (ret == ESP_ERR_TIMEOUT) {
			memcpy(&mag, &last_mag, sizeof(mag));
		} else if (ret != ESP_OK) {
			ESP_LOGE(__FILE__, "Could not read mag data from mpu9250");
		} else {
			last_mag = mag;
		}
		drv_ak8963_correct_data(&mag);

		#if defined(GPS_MAG)
			drv_hmc5883_data_t compass_data;
			memset(&compass_data, 0, sizeof(compass_data));
			drv_hmc5883_get_data(&compass_data);
			drv_hmc5883_correct_data(&compass_data);
			mag.x = compass_data.x;
			mag.y = compass_data.y;
			mag.z = compass_data.z;
		#endif

		// printf("acc x %10.5f, y %10.5f, z %10.5f ", acc.x, acc.y, acc.z);
		// printf("gyr x %10.5f, y %10.5f, z %10.5f ", gyr.x, gyr.y, gyr.z);
		// printf("mag x %10.5f, y %10.5f, z %10.5f %10.5f \n", mag.x, mag.y, mag.z, get_time_s());

		/* Correct mag data for distortion by main current */
		// compensate_mag_with_current(psu_data.sense_main_curr, &mag);

		static float last_time = 0.0;
		float delta_time = get_time_s() - last_time;
		last_time = get_time_s();

		double datt[3] = {0.0};
		
		double dacc[3] = {acc.x,acc.y,acc.z};
		double dgyr[3] = {gyr.x,gyr.y,gyr.z};
		double dmag[3] = {mag.x,mag.y,mag.z};

		algo_attitude_ts64(q_att,b_gyr,dacc,dgyr,dmag,K_ACC,K_MAG,K_BIAS_ACC,K_BIAS_MAG,delta_time);
		quaternion_euler_conv64(q_att,datt);
		float att[3] = {datt[0],datt[1],datt[2]};
		// attitude_est((float*)&acc, (float*)&gyr, (float*)&mag, att, delta_time);
		// printf("Att: %10.5f, %10.5f, %10.5f \n", att[0], att[1], att[2]);

		float temp = bmp280_get_temp();
		float press = bmp280_get_press();
		// printf("Temp: %10.2f, Press: %5.2f \n", temp, press);

		if (xSemaphoreTake(ahrs_sem, 10 / portTICK_PERIOD_MS) == true) {

			ahrs_data.time = last_time;

			ahrs_data.acc[0] = acc.x;
			ahrs_data.acc[1] = acc.y;
			ahrs_data.acc[2] = acc.z;

			ahrs_data.gyr[0] = gyr.x;
			ahrs_data.gyr[1] = gyr.y;
			ahrs_data.gyr[2] = gyr.z;

			ahrs_data.mag[0] = mag.x;
			ahrs_data.mag[1] = mag.y;
			ahrs_data.mag[2] = mag.z;

			ahrs_data.att[0] = att[0];
			ahrs_data.att[1] = att[1];
			ahrs_data.att[2] = att[2];

			ahrs_data.temp = temp;
			ahrs_data.press = press;

			xSemaphoreGive(ahrs_sem);

		} else {
			ESP_LOGW(__FILE__, "AHRS worker could not lock mutex");
		}

		delay_until_ms(&last_wake_time, 10);
	}
}

void attitude_test() {

	ESP_LOGI(__FILE__, "Attitude Test");

	// mpu9250_acc_data_t acc_err;
	// mpu9250_gyr_data_t gyr_err;
	// ak8963_mag_data_t mag_err;
	//
	// ESP_LOGI(__FILE__, "Attitude Calibrating Sensors ...");
	//
	// for (uint32_t i=0; i<NUM_CAL_SAMPLES; i++) {
	//
	// 	mpu9250_acc_data_t acc_raw;
	// 	mpu9250_gyr_data_t gyr_raw;
	// 	ak8963_mag_data_t mag_raw;
	//
	// 	drv_mpu9250_read_acc(&acc_raw);
	// 	drv_mpu9250_read_gyr(&gyr_raw);
	// 	drv_ak8963_read_mag(&mag_raw);
	//
	// 	acc_err.x += acc_raw.x;
	// 	acc_err.y += acc_raw.y;
	// 	acc_err.z += acc_raw.z - 1.0;
	// 	gyr_err.x += gyr_raw.x;
	// 	gyr_err.y += gyr_raw.y;
	// 	gyr_err.z += gyr_raw.z;
	// 	mag_err.x += mag_raw.x;
	// 	mag_err.y += mag_raw.y;
	// 	mag_err.z += mag_raw.z;
	//
	// 	delay_ms(CAL_DELAY);
	// }
	//
	// acc_err.x /= NUM_CAL_SAMPLES;
	// acc_err.y /= NUM_CAL_SAMPLES;
	// acc_err.z /= NUM_CAL_SAMPLES;
	// gyr_err.x /= NUM_CAL_SAMPLES;
	// gyr_err.y /= NUM_CAL_SAMPLES;
	// gyr_err.z /= NUM_CAL_SAMPLES;
	// mag_err.x /= NUM_CAL_SAMPLES;
	// mag_err.y /= NUM_CAL_SAMPLES;
	// mag_err.z /= NUM_CAL_SAMPLES;

	// while(1) {
	//
	// 	float acc[3];
	// 	float gyr[3];
	// 	float mag[3];
	//
	// 	mpu9250_acc_data_t acc_raw;
	// 	mpu9250_gyr_data_t gyr_raw;
	// 	ak8963_mag_data_t mag_raw;
	//
	// 	drv_mpu9250_read_acc(&acc_raw);
	// 	drv_mpu9250_read_gyr(&gyr_raw);
	// 	drv_ak8963_read_mag(&mag_raw);
	//
	// 	acc[0] = (acc_raw.x - acc_err.x);
	// 	acc[1] = (acc_raw.y - acc_err.y);
	// 	acc[2] = (acc_raw.z - acc_err.z);
	// 	gyr[0] = (gyr_raw.x - gyr_err.x)*PI/180.0;
	// 	gyr[1] = (gyr_raw.y - gyr_err.y)*PI/180.0;
	// 	gyr[2] = (gyr_raw.z - gyr_err.z)*PI/180.0;
	// 	mag[0] = (mag_raw.x /*- mag_err.x*/)/100.0;
	// 	mag[1] = (mag_raw.y /*- mag_err.y*/)/100.0;
	// 	mag[2] = (mag_raw.z /*- mag_err.z*/)/100.0;
	//
	// 	printf("Acc: %8.5f, %8.5f, %8.5f, ", acc[0], acc[1], acc[2]);
	// 	printf("Gyr: %8.5f, %8.5f, %8.5f, ", gyr[0], gyr[1], gyr[2]);
	// 	printf("Mag: %8.5f, %8.5f, %8.5f, ", mag[0], mag[1], mag[2]);
	//
	// 	static float last_time;
	// 	float delta_time = get_time_s() - last_time;
	// 	last_time = get_time_s();
	//
	// 	float att[3];
	// 	attitude_est(acc, gyr, mag, att, delta_time);
	// 	printf("Att: %10.5f, %10.5f, %10.5f \n", att[0]*180/PI, att[1]*180/PI, att[2]*180/PI);
	//
	// 	// printf("Delta Time: %.4f \n", delta_time);
	//
	// 	// printf("Temp: %.2f \n", drv_mpu9250_read_temp());
	//
	// 	// mpu9250_acc_data_t acc;
	// 	// drv_mpu9250_read_acc(&acc);
	// 	// printf("acc x %10.5f, y %10.5f, z %10.5f ", acc.x, acc.y, acc.z);
	// 	//
	// 	// mpu9250_gyr_data_t gyr;
	// 	// drv_mpu9250_read_gyr(&gyr);
	// 	// printf("gyr x %10.5f, y %10.5f, z %10.5f ", gyr.x, gyr.y, gyr.z);
	// 	//
	// 	// ak8963_mag_data_t mag;
	// 	// drv_ak8963_read_mag(&mag);
	// 	// printf("mag x %10.5f, y %10.5f, z %10.5f %10.5f ", mag.x, mag.y, mag.z, get_time_s());
	//
	// 	// printf("Press: %5.2f, Temp: %5.2f \n", bmp280_get_press(), bmp280_get_temp());
	//
	// 	delay_ms(10);
	// }
}
