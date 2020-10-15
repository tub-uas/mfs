#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "drv_nema_parser.h"

#define TIME_ZONE (+1)   // Berlin
#define YEAR_BASE (2000) // Date in GPS starts from 2000


static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    gps_t *gps = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(__FILE__, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(__FILE__, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

void main_gps() {
    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    /* unregister event handler */
    nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    /* deinit NMEA parser library */
    nmea_parser_deinit(nmea_hdl);
}


// #include "main_gps.h"
//
// #include <stdio.h>
// #include <stdint.h>
// #include <stdlib.h>
// #include <string.h>
//
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_err.h"
// #include "driver/gpio.h"
//
// #include "drv_button.h"
// #include "drv_can.h"
// #include "drv_led.h"
// #include "drv_sense.h"
// #include "util.h"
//
//
// void main_gps() {
//
// 	ESP_LOGI(__FILE__, "Running main_gps");
//
// 	TickType_t last_wake_time = xTaskGetTickCount();
//
// 	while (1) {
//
// 		drv_led_set(LED_ON_ALIVE);
//
// 		printf("hello biatch %f \n", get_time_s());
//
// 		delay_until_ms(&last_wake_time, 10);
// 	}
//
// 	// If we get here, something went badly wrong. Reset the system
// 	// TODO set the failsafe mode
// 	esp_restart();
// }
