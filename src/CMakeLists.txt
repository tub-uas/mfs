cmake_minimum_required(VERSION 3.16.0)

set(app_sources
	"attitude_estimation.c"
	"board.c"
	"can_com_ahrs.c"
	"can_com_gps.c"
	"can_com_rai.c"
	"can_com_psu.c"
	"drv_ak8963.c"
	"drv_bmp280.c"
	"drv_button.c"
	"drv_can.c"
	"drv_gps.c"
	"drv_hmc5883.c"
	"drv_i2c.c"
	"drv_led.c"
	"drv_mpu9250.c"
	"drv_nmea.c"
	"drv_pwm.c"
	"drv_sense_ext.c"
	"drv_sense.c"
	"drv_sumd.c"
	"main_ahrs.c"
	"main_gps.c"
	"main_psu.c"
	"main_rai.c"
	"main.c"
	"pid.c"
	"system.c"
	"util_quaternion.c"
	"util_vecmath.c"
	"util.c"
	"util_quaternion64.c"
	"util_math64.c"
	"algo_attitude64.c"
)

idf_component_register(SRCS ${app_sources})
