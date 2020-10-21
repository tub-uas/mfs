#ifndef CAN_COM_GPS_H
#define CAN_COM_GPS_H

#include <stdint.h>

#include "esp_err.h"

typedef struct {
	float time;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	uint8_t fix;
	uint8_t fix_mode;
	uint8_t sats_in_view;
	uint8_t sats_in_use;
	float latitude;
	float longitude;
	float altitude;
	float speed;
	float cog;
	float dop_h;
	float dop_p;
	float dop_v;
	float variation;
	uint8_t valid;
} can_com_gps_t;

esp_err_t can_com_gps_init();

esp_err_t can_com_gps_send(can_com_gps_t data);

#endif // CAN_COM_GPS_H
