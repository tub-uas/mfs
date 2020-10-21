#include "main_gps.h"

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "util.h"
#include "drv_nmea.h"
#include "drv_led.h"

#define TIME_ZONE (+2)   // Berlin
#define YEAR_BASE (2000) // Date in GPS starts from 2000

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	gps_t *gps = NULL;
	switch (event_id) {
		case GPS_UPDATE:
			gps = (gps_t *)event_data;
			/* print information parsed from GPS statements */
			ESP_LOGI(__FILE__, "%d/%d/%d %d:%d:%d => \r\n"
					"\t\t\t\t\t\tlatitude     = %.07f°N\r\n"
					"\t\t\t\t\t\tlongitude    = %.07f°E\r\n"
					"\t\t\t\t\t\taltitude     = %.02fm\r\n"
					"\t\t\t\t\t\tspeed        = %.05fm/s\r\n"
					"\t\t\t\t\t\tfix          = %d\r\n"
					"\t\t\t\t\t\tfix_mode     = %d\r\n"
					"\t\t\t\t\t\tsats_in_view = %d\r\n"
					"\t\t\t\t\t\tsats_in_use  = %d\r\n"
					"\t\t\t\t\t\tdop_h        = %f\r\n"
					"\t\t\t\t\t\tdop_p        = %f\r\n"
					"\t\t\t\t\t\tdop_v        = %f\r\n"
					"\t\t\t\t\t\tvariation    = %f\r\n"
					"\t\t\t\t\t\tcog          = %f\r\n"
					"\t\t\t\t\t\tvalid        = %d\r\n",
					gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
					gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
					gps->latitude,
					gps->longitude,
					gps->altitude,
					gps->speed,
					gps->fix,
					gps->fix_mode,
					gps->sats_in_view,
					gps->sats_in_use,
					gps->dop_h,
					gps->dop_p,
					gps->dop_v,
					gps->variation,
					gps->cog,
					gps->valid);
			break;

		case GPS_UNKNOWN:
		default:
			/* print unknown statements */
			ESP_LOGW(__FILE__, "Unknown statement:%s", (char *)event_data);
			break;
	}
}

void main_gps() {

	ESP_LOGI(__FILE__, "Running main_gps");

	/* NMEA parser configuration */
	nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();

	/* init NMEA parser library */
	nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);

	/* register event handler for NMEA parser library */
	ESP_ERROR_CHECK(nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL));

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {

		drv_led_set(LED_ON_ALIVE);

		delay_until_ms(&last_wake_time, 10);
	}

	// If we get here, something went badly wrong. Reset the system
	// TODO set the failsafe mode
	esp_restart();

	/* unregister event handler */
	nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
	/* deinit NMEA parser library */
	nmea_parser_deinit(nmea_hdl);

}
