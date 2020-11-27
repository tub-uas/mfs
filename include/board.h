#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include "esp_err.h"

/* Orientation in which the AHRS board is in */
// #define NATIVE_ORIENT
#define DEFAULT_ORIENT
// #define HYPE_ORIENT

uint32_t board_get_id();

esp_err_t board_print_id();

esp_err_t board_check();

#endif // BOARD_H
