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
#include "system.h"
#include "util_quaternion.h"
#include "util_vecmath.h"
#include "util.h"

void app_main() {

	delay_ms(10); /* Let the system settle down */

	ESP_LOGI(__FILE__, "================= Starting System =================");

	ESP_ERROR_CHECK(board_check()); /* Check that software matches hardware */

	#if defined(RAI_BOARD)
		ESP_LOGI(__FILE__, "Board type is RAI");
		ESP_ERROR_CHECK(drv_led_init()); /* Starts LED worker */
		ESP_ERROR_CHECK(drv_button_init()); /* Configure user button */
		ESP_ERROR_CHECK(drv_sense_init()); /* Starts local voltage monitoring */
		ESP_ERROR_CHECK(drv_pwm_init()); /* Configure pwm servo outputs */
		ESP_ERROR_CHECK(drv_sumd_init()); /* Starts SUMD worker */
		ESP_ERROR_CHECK(can_com_rai_init()); /* Init rai can interface */

	#elif defined(PSU_BOARD)
		ESP_LOGI(__FILE__, "Board type is PSU");
		ESP_ERROR_CHECK(drv_led_init()); /* Starts LED worker */
		ESP_ERROR_CHECK(drv_button_init()); /* Configure user button */
		ESP_ERROR_CHECK(drv_sense_ext_init()); /* Starts psu voltage and current meas. */
		ESP_ERROR_CHECK(can_com_psu_init(0)); /* Only transmit psu data */

	#elif defined(AHRS_BOARD)
		ESP_LOGI(__FILE__, "Board type is AHRS");
		ESP_ERROR_CHECK(drv_led_init()); /* Starts LED worker */
		ESP_ERROR_CHECK(drv_button_init()); /* Configure user button */
		ESP_ERROR_CHECK(drv_sense_init()); /* Starts local voltage monitoring */
		ESP_ERROR_CHECK(can_com_psu_init(1)); /* Is able to receive psu data */
		ESP_ERROR_CHECK(drv_mpu9250_init()); /* Accel and gyro init */
		ESP_ERROR_CHECK(drv_ak8963_init()); /* Magnetometer init */
		ESP_ERROR_CHECK(drv_bmp280_init()); /* Barometer init */
		ESP_ERROR_CHECK(can_com_ahrs_init()); /* Init ahrs can interface */
		// ESP_ERROR_CHECK(attitude_init()); /* Starts attitude worker */

	#elif defined(GPS_BOARD)
		ESP_LOGI(__FILE__, "Board type is GPS");
		ESP_ERROR_CHECK(drv_led_init()); /* Starts LED worker */
		ESP_ERROR_CHECK(drv_button_init()); /* Configure user button */
		ESP_ERROR_CHECK(drv_sense_init()); /* Starts local voltage monitoring */
		ESP_ERROR_CHECK(can_com_gps_init()); /* Init gps can interface */

	#endif

	ESP_LOGI(__FILE__, "=========== System Started Successfully ===========");

	#if defined(RAI_BOARD)
		xTaskCreate(send_rai, "send_rai", 4096, NULL, 10, NULL);
		xTaskCreate(recv_rai, "recv_rai", 4096, NULL, 20, NULL);

	#elif defined(PSU_BOARD)
		xTaskCreate(main_psu, "main_psu", 4096, NULL, 10, NULL);

	#elif defined(AHRS_BOARD)
		xTaskCreate(main_ahrs, "main_ahrs", 4096, NULL, 10, NULL);

	#elif defined(GPS_BOARD)
		xTaskCreate(main_gps, "main_gps", 4096, NULL, 10, NULL);

	#endif
}
