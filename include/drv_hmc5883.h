#ifndef DRV_HMC5883_H
#define DRV_HMC5883_H

#include <stdint.h>

#include "esp_err.h"

#define HMC5883_ADDR          0x1E
#define HMC5883_WHO_AM_I_RESP 0x00333448
#define HMC5883_TIMEOUT       100

typedef enum {
	HMC5883L_MODE_CONTINUOUS = 0, //!< Continuous mode
	HMC5883L_MODE_SINGLE          //!< Single measurement mode, default
} drv_hmc5883_opmode_t;

typedef enum {
	HMC5883L_SAMPLES_1 = 0, //!< 1 sample, default
	HMC5883L_SAMPLES_2,     //!< 2 samples
	HMC5883L_SAMPLES_4,     //!< 4 samples
	HMC5883L_SAMPLES_8      //!< 8 samples
} drv_hmc5883_samples_averaged_t;

typedef enum {
	HMC5883L_DATA_RATE_00_75 = 0, //!< 0.75 Hz
	HMC5883L_DATA_RATE_01_50,     //!< 1.5 Hz
	HMC5883L_DATA_RATE_03_00,     //!< 3 Hz
	HMC5883L_DATA_RATE_07_50,     //!< 7.5 Hz
	HMC5883L_DATA_RATE_15_00,     //!< 15 Hz, default
	HMC5883L_DATA_RATE_30_00,     //!< 30 Hz
	HMC5883L_DATA_RATE_75_00      //!< 75 Hz
} drv_hmc5883_data_rate_t;

typedef enum {
	HMC5883L_BIAS_NORMAL = 0, //!< Default flow, no bias
	HMC5883L_BIAS_POSITIVE,   //!< Positive bias configuration all axes, used for self test (see datasheet)
	HMC5883L_BIAS_NEGATIVE    //!< Negative bias configuration all axes, used for self test (see datasheet)
} drv_hmc5883_bias_t;

typedef enum {
	HMC5883L_GAIN_1370 = 0, //!< 0.73 mG/LSb, range -0.88..+0.88 G
	HMC5883L_GAIN_1090,     //!< 0.92 mG/LSb, range -1.3..+1.3 G, default
	HMC5883L_GAIN_820,      //!< 1.22 mG/LSb, range -1.9..+1.9 G
	HMC5883L_GAIN_660,      //!< 1.52 mG/LSb, range -2.5..+2.5 G
	HMC5883L_GAIN_440,      //!< 2.27 mG/LSb, range -4.0..+4.0 G
	HMC5883L_GAIN_390,      //!< 2.56 mG/LSb, range -4.7..+4.7 G
	HMC5883L_GAIN_330,      //!< 3.03 mG/LSb, range -5.6..+5.6 G
	HMC5883L_GAIN_230       //!< 4.35 mG/LSb, range -8.1..+8.1 G
} drv_hmc5883_gain_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} drv_hmc5883_raw_data_t;

typedef struct {
	float x;
	float y;
	float z;
} drv_hmc5883_data_t;

esp_err_t drv_hmc5883_init();

esp_err_t drv_hmc5883_ping();

esp_err_t drv_hmc5883_get_opmode(drv_hmc5883_opmode_t *val);

esp_err_t drv_hmc5883_set_opmode(drv_hmc5883_opmode_t mode);

esp_err_t drv_hmc5883_get_samples_averaged(drv_hmc5883_samples_averaged_t *val);

esp_err_t drv_hmc5883_set_samples_averaged(drv_hmc5883_samples_averaged_t samples);

esp_err_t drv_hmc5883_get_data_rate(drv_hmc5883_data_rate_t *val);

esp_err_t drv_hmc5883_set_data_rate(drv_hmc5883_data_rate_t rate);

esp_err_t drv_hmc5883_get_bias(drv_hmc5883_bias_t *val);

esp_err_t drv_hmc5883_set_bias(drv_hmc5883_bias_t bias);

esp_err_t drv_hmc5883_get_gain(drv_hmc5883_gain_t *val);

esp_err_t drv_hmc5883_set_gain(drv_hmc5883_gain_t gain);

esp_err_t drv_hmc5883_data_is_locked(uint8_t *val);

esp_err_t drv_hmc5883_data_is_ready(uint8_t *val);

esp_err_t drv_hmc5883_get_raw_data(drv_hmc5883_raw_data_t *data);

esp_err_t drv_hmc5883_raw_to_mt(drv_hmc5883_raw_data_t raw, drv_hmc5883_data_t *mt);

esp_err_t drv_hmc5883_get_data(drv_hmc5883_data_t *data);

esp_err_t drv_hmc5883_correct_data(drv_hmc5883_data_t *data);

#endif // DRV_HMC5883_H
