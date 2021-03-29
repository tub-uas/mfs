#include "main_gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "util.h"
#include "drv_nmea.h"
#include "drv_hmc5883.h"
#include "drv_led.h"
#include "can_com_gps.h"

#define TIME_ZONE (+2)   // Berlin

static atomic_uint last_valid_sample_time = 0;

static can_com_gps_t data;
static SemaphoreHandle_t gps_data_sem = NULL;

/* When all GPS messages have been received this callback gets executed. */
static void gps_event_handler(void *event_handler_arg,
                              esp_event_base_t event_base,
                              int32_t event_id,
                              void *event_data) {
	gps_t *gps = NULL;
	switch (event_id) {
		case GPS_UPDATE:

			gps = (gps_t *)event_data;

			// printf("\n--- %d/%d/%d %d:%d:%d ---\n"
			//        "latitude     = %.07fN\n"
			//        "longitude    = %.07fE\n"
			//        "altitude     = %.02fm\n"
			//        "speed        = %.05fm/s\n"
			//        "fix          = %d\n"
			//        "fix_mode     = %d\n"
			//        "sats_in_view = %d\n"
			//        "sats_in_use  = %d\n"
			//        "dop_h        = %f\n"
			//        "dop_p        = %f\n"
			//        "dop_v        = %f\n"
			//        "variation    = %f\n"
			//        "cog          = %f\n"
			//        "valid        = %d\n",
			//        gps->date.year + 2000, gps->date.month, gps->date.day,
			//        gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
			//        gps->latitude,
			//        gps->longitude,
			//        gps->altitude,
			//        gps->speed,
			//        gps->fix,
			//        gps->fix_mode,
			//        gps->sats_in_view,
			//        gps->sats_in_use,
			//        gps->dop_h,
			//        gps->dop_p,
			//        gps->dop_v,
			//        gps->variation,
			//        gps->cog,
			//        gps->valid);

			if (xSemaphoreTake(gps_data_sem, 10 / portTICK_PERIOD_MS) == true) {

				data.time = get_time_s();
				data.second = gps->tim.second;
				data.minute = gps->tim.minute;
				data.hour = gps->tim.hour + TIME_ZONE;
				data.day = gps->date.day;
				data.month = gps->date.month;
				data.year = gps->date.year + 2000;
				data.fix = gps->fix;
				data.fix_mode = gps->fix_mode;
				data.sats_in_view = gps->sats_in_view;
				data.sats_in_use = gps->sats_in_use;
				data.latitude = gps->latitude;
				data.longitude = gps->longitude;
				data.altitude = gps->altitude;
				data.speed = gps->speed;
				data.cog = gps->cog;
				data.dop_h = gps->dop_h;
				data.dop_p = gps->dop_p;
				data.dop_v = gps->dop_v;
				data.variation = gps->variation;
				data.valid = gps->valid;

				xSemaphoreGive(gps_data_sem);

			} else {
				ESP_LOGW(__FILE__, "GPS data sem timeout at mag");
			}

			if (gps->valid > 0 && gps->fix > 0 && gps->fix_mode > 1) {
				last_valid_sample_time = get_time_ms();
			}

			break;

		case GPS_UNKNOWN:
		default:
			/* print unknown statements */
			ESP_LOGW(__FILE__, "Unknown statement:%s", (char *)event_data);
			break;
	}
}

void main_gps() {

	ESP_LOGI(__FILE__, "RUNNING MAIN_GPS");

	/* Wait for the GPS to start up */
	delay_ms(1000);

	/* Reset GPS data */
	memset(&data, 0, sizeof(data));

	/* Initialize GPS mutex */
	gps_data_sem = xSemaphoreCreateMutex();
	if (gps_data_sem == NULL) {
		ESP_LOGE(__FILE__, "Cant create GPS data mutex");
	}

	/* NMEA parser configuration */
	nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();

	/* Init NMEA parser library */
	ESP_LOGI(__FILE__, "Second connection with GPS in operation mode");
	nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);

	/* register event handler for NMEA parser library */
	ESP_ERROR_CHECK(nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL));

	TickType_t last_wake_time = xTaskGetTickCount();

	while (1) {

		drv_hmc5883_data_t compass_data;
		memset(&compass_data, 0, sizeof(compass_data));
		drv_hmc5883_get_data(&compass_data);

		// printf("compass_x = %10.5f   "
		//        "compass_y = %10.5f   "
		//        "compass_z = %10.5f  \n",
		//        compass_data.x,
		//        compass_data.y,
		//        compass_data.z);

		if (xSemaphoreTake(gps_data_sem, 10 / portTICK_PERIOD_MS) == true) {
			data.mag_x = compass_data.x;
			data.mag_y = compass_data.y;
			data.mag_z = compass_data.z;

			// Send GPS data over CAN
			can_com_gps_send(data);

			xSemaphoreGive(gps_data_sem);

		} else {
			ESP_LOGW(__FILE__, "GPS data sem timeout at mag");
		}

		if (get_time_ms() < last_valid_sample_time + 2000) {
			drv_led_set(LED_ON_ALIVE);
		} else {
			drv_led_set(LED_FAST);
		}

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
