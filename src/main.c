#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "attitude_estimation.h"
#include "board.h"
#include "can_com_ahrs.h"
#include "can_com_gps.h"
#include "can_com_psu.h"
#include "can_com_rai.h"
#include "can_ids.h"
#include "can_meta.h"
#include "drv_ak_regs.h"
#include "drv_ak8963.h"
#include "drv_bmp_regs.h"
#include "drv_bmp280.h"
#include "drv_button.h"
#include "drv_can.h"
#include "drv_i2c.h"
#include "drv_led.h"
#include "drv_mpu_regs.h"
#include "drv_mpu9250.h"
#include "drv_nmea.h"
#include "drv_pwm.h"
#include "drv_sense_ext.h"
#include "drv_sense.h"
#include "drv_sumd.h"
#include "main_ahrs.h"
#include "main_gps.h"
#include "main_psu.h"
#include "main_rai.h"
#include "pid.h"
#include "util_quaternion.h"
#include "util_vecmath.h"
#include "util.h"

void app_main() {

	delay_ms(100);

	ESP_LOGI(__FILE__, "================= Starting System =================");

	// ESP_ERROR_CHECK(board_check());

	#if defined(RAI_BOARD)
		ESP_LOGI(__FILE__, "Board type is RAI");
		ESP_ERROR_CHECK(drv_led_init());
		ESP_ERROR_CHECK(drv_button_init());
		ESP_ERROR_CHECK(drv_sense_init());
		ESP_ERROR_CHECK(drv_pwm_init());
		ESP_ERROR_CHECK(drv_sumd_init());
		ESP_ERROR_CHECK(can_com_rai_init());

	#elif defined(PSU_BOARD)
		ESP_LOGI(__FILE__, "Board type is PSU");
		ESP_ERROR_CHECK(drv_led_init());
		ESP_ERROR_CHECK(drv_button_init());
		ESP_ERROR_CHECK(drv_sense_ext_init());
		ESP_ERROR_CHECK(can_com_psu_init());

	#elif defined(AHRS_BOARD)
		ESP_LOGI(__FILE__, "Board type is AHRS");
		ESP_ERROR_CHECK(drv_led_init());
		ESP_ERROR_CHECK(drv_button_init());
		ESP_ERROR_CHECK(drv_sense_init());
		ESP_ERROR_CHECK(drv_mpu9250_init());
		ESP_ERROR_CHECK(drv_ak8963_init());
		ESP_ERROR_CHECK(drv_bmp280_init());
		ESP_ERROR_CHECK(can_com_ahrs_init());
		ESP_ERROR_CHECK(attitude_init());

	#elif defined(GPS_BOARD)
		ESP_LOGI(__FILE__, "Board type is GPS");
		ESP_ERROR_CHECK(drv_led_init());
		ESP_ERROR_CHECK(drv_button_init());
		ESP_ERROR_CHECK(drv_sense_init());
		ESP_ERROR_CHECK(can_com_gps_init());
		// ESP_ERROR_CHECK(drv_nema_init());

	#endif

	ESP_LOGI(__FILE__, "=========== System Started Successfully ===========");

	#if defined(RAI_BOARD)
		xTaskCreate(send_rai, "send_rai", 4096, NULL, 5, NULL);
		xTaskCreate(recv_rai, "recv_rai", 4096, NULL, 5, NULL);

	#elif defined(PSU_BOARD)
		xTaskCreate(main_psu, "main_psu", 4096, NULL, 5, NULL);

	#elif defined(AHRS_BOARD)
		xTaskCreate(main_ahrs, "main_ahrs", 4096, NULL, 5, NULL);

	#elif defined(GPS_BOARD)
		// xTaskCreate(main_gps, "main_gps", 4096, NULL, 5, NULL);
		xTaskCreate(main_gps_compass, "main_gps_compass", 4096, NULL, 5, NULL);
		// xTaskCreate(drv_nema_test, "drv_nema_test", 4096, NULL, 5, NULL);

	#endif

	// xTaskCreate(attitude_test, "attitude_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_ak8963_test, "drv_ak8963_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_bmp280_test, "drv_bmp280_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_button_test, "drv_button_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_can_test, "drv_can_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_mpu9250_test, "drv_mpu9250_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_led_test, "drv_led_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_pwm_test, "drv_pwm_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_sense_ext_test, "drv_sense_ext_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_sense_test, "drv_sense_test", 4096, NULL, 5, NULL);
	// xTaskCreate(drv_sumd_test, "drv_sumd_test", 4096, NULL, 5, NULL);

	// xTaskCreate(hype_test_sevo, "hype_test_sevo", 4096, NULL, 5, NULL);
	// xTaskCreate(hype_test_voltage, "hype_test_voltage", 4096, NULL, 5, NULL);

}
