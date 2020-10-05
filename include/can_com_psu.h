#ifndef CAN_COM_PSU_H
#define CAN_COM_PSU_H

#include <stdint.h>

#include "esp_err.h"

typedef struct {
	float sense_main_volt;
	float sense_main_curr;
	float sense_main_pow;
	float sense_pwr_volt;
	float sense_pwr_curr;
	float sense_pwr_pow;
	float sense_sys_volt;
	float sense_sys_curr;
	float sense_sys_pow;
	float sense_time;
} can_com_psu_t;

esp_err_t can_com_psu_init();

esp_err_t can_com_psu_send(can_com_psu_t data);

#endif // CAN_COM_PSU_H
