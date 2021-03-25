#include "board.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "util.h"

#define NUM_GENS 2

#define RAI_GEN1    0xbf10666c
#define PSU_GEN1    0xbf106be0
#define AHRS_GEN1   0xbf106614
#define GPS_GEN1    0xbf106688

#define RAI_GEN2    0xb21c6684
#define PSU_GEN2    0xb21e0844
#define AHRS_GEN2   0x33692cb5 // combines AHRS and GPS from GEN1
#define GPS_GEN2    0xffffffff

#if defined(RAI_BOARD)
	uint32_t valid_ids[NUM_GENS] = {RAI_GEN1, RAI_GEN2};
	#define BOARD_NAME "RAI"
#elif defined(PSU_BOARD)
	uint32_t valid_ids[NUM_GENS] = {PSU_GEN1, PSU_GEN2};
	#define BOARD_NAME "PSU"
#elif defined(AHRS_BOARD)
	uint32_t valid_ids[NUM_GENS] = {AHRS_GEN1, AHRS_GEN2};
	#define BOARD_NAME "AHRS"
#elif defined(GPS_BOARD)
	uint32_t valid_ids[NUM_GENS] = {GPS_GEN1, GPS_GEN2};
	#define BOARD_NAME "GPS"
#else
	#error "Unkown board type"
#endif

uint32_t board_get_id() {
	uint8_t mac[6];
	esp_read_mac(mac, ESP_MAC_WIFI_STA);
	// printf("%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],
	//                                          mac[1],
	//                                          mac[2],
	//                                          mac[3],
	//                                          mac[4],
	//                                          mac[5]);

	return ((uint32_t)mac[2]<<24) |
	       ((uint32_t)mac[3]<<16) |
	       ((uint32_t)mac[4]<<8)  |
	       ((uint32_t)mac[5]);
}

esp_err_t board_print_id() {
	printf("Board ID is: 0x%8x \n", board_get_id());
	return ESP_OK;
}

esp_err_t board_check() {

	for (uint32_t i=0; i<NUM_GENS; i++) {
		if (board_get_id() == valid_ids[i]) {
			return ESP_OK;
		}
	}

	ESP_LOGE(__FILE__, "Board type does not match software type %s", BOARD_NAME);

	return ESP_FAIL;
}
