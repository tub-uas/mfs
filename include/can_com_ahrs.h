#ifndef CAN_COM_AHRS_H
#define CAN_COM_AHRS_H

#include <stdint.h>

#include "esp_err.h"

typedef struct {
	float time;
	float acc[3];
	float gyr[3];
	float mag[3];
	float att[3];
	float temp;
	float press;
} can_com_ahrs_t;

esp_err_t can_com_ahrs_init();

esp_err_t can_com_ahrs_send(can_com_ahrs_t data);

#endif // CAN_COM_AHRS_H
