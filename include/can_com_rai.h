#ifndef CAN_COM_RAI_H
#define CAN_COM_RAI_H

#include <stdint.h>

#include "esp_err.h"

typedef struct {
	uint16_t chnl[12];
	float time;
} can_com_rai_t;

esp_err_t can_com_rai_init();

esp_err_t can_com_rai_send(can_com_rai_t data);

esp_err_t can_com_rai_get(can_com_rai_t *data);

esp_err_t can_com_rai_recv(can_com_rai_t *data);

#endif // CAN_COM_RAI_H
