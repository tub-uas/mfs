#ifndef DRV_BMP280_H
#define DRV_BMP280_H

#include <stdint.h>

#include "esp_err.h"

#define BMP280_ADDR          0x76
#define BMP280_WHO_AM_I_RESP 0x58
#define BMP280_TIMEOUT       100

typedef enum {
	FILTOFF     = 0x00,
	FILT_X2     = 0x01,
	FILT_X4     = 0x02,
	FILT_X8     = 0x03,
	FILT_X16    = 0x04,
} bmp280_filter_t;

typedef enum {
	PNO_SMPLE    = 0x00,
	PSMPLE_X1    = 0x01,
	PSMPLE_X2    = 0x02,
	PSMPLE_X4    = 0x03,
	PSMPLE_X8    = 0x04,
	PSMPLE_X16   = 0x05,
} bmp280_ovrs_press_t;

typedef enum {
	TNO_SMPLE    = 0x00,
	TSMPLE_X1    = 0x01,
	TSMPLE_X2    = 0x02,
	TSMPLE_X4    = 0x03,
	TSMPLE_X8    = 0x04,
	TSMPLE_X16   = 0x05,
} bmp280_ovrs_temp_t;

typedef enum {
	MODE_SLEEP      = 0x00,
	MODE_FORCED     = 0x01,
	MODE_NORMAL     = 0x03,
	MODE_SOFTRST    = 0xB6,
} bmp280_pwrm_t;

typedef enum {
	MS00005 = 0x00,
	MS00625 = 0x01,
	MS01250 = 0x02,
	MS02500 = 0x03,
	MS05000 = 0x04,
	MS10000 = 0x05,
	MS20000 = 0x06,
	MS40000 = 0x07,
} bmp280_stdby_t;

typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} bmp280_calib_t;

typedef struct {
	double temp;
	double press;
} bmp280_data_t;

typedef struct {
	bmp280_pwrm_t       pwr_mode;
	bmp280_stdby_t      stdby_time;
	bmp280_ovrs_temp_t  ovrs_temp;
	bmp280_ovrs_press_t ovrs_press;
	bmp280_filter_t     filter_set;
} bmp280_init_t;

esp_err_t drv_bmp280_init();

esp_err_t drv_bmp280_ping();

esp_err_t drv_bmp280_reset();

esp_err_t drv_bmp280_set_filter(bmp280_filter_t filt);

esp_err_t drv_bmp280_set_ovrs_press(bmp280_ovrs_press_t ovrs_press);

esp_err_t drv_bmp280_set_ovrs_temp(bmp280_ovrs_temp_t ovrs_temp);

esp_err_t drv_bmp280_set_mode(bmp280_pwrm_t mode);

esp_err_t drv_bmp280_set_standby_duration(bmp280_stdby_t stdby);

esp_err_t drv_bmp280_set_config_reg(bmp280_init_t init);

esp_err_t drv_bmp280_set_ctrl_meas_reg(bmp280_init_t init);

esp_err_t drv_bmp280_read_coeff(bmp280_calib_t *cal);

double bmp280_get_temp();

double bmp280_get_temp_raw(int32_t *t_data, bmp280_calib_t cal);

double bmp280_get_press();

double bmp280_get_press_raw(int32_t *t_data, bmp280_calib_t cal);

void drv_bmp280_test();

#endif // DRV_BMP280_H
