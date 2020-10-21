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
} hmc5883l_opmode_t;

typedef enum {
	HMC5883L_SAMPLES_1 = 0, //!< 1 sample, default
	HMC5883L_SAMPLES_2,     //!< 2 samples
	HMC5883L_SAMPLES_4,     //!< 4 samples
	HMC5883L_SAMPLES_8      //!< 8 samples
} hmc5883l_samples_averaged_t;

typedef enum {
	HMC5883L_DATA_RATE_00_75 = 0, //!< 0.75 Hz
	HMC5883L_DATA_RATE_01_50,     //!< 1.5 Hz
	HMC5883L_DATA_RATE_03_00,     //!< 3 Hz
	HMC5883L_DATA_RATE_07_50,     //!< 7.5 Hz
	HMC5883L_DATA_RATE_15_00,     //!< 15 Hz, default
	HMC5883L_DATA_RATE_30_00,     //!< 30 Hz
	HMC5883L_DATA_RATE_75_00      //!< 75 Hz
} hmc5883l_data_rate_t;

typedef enum {
	HMC5883L_BIAS_NORMAL = 0, //!< Default flow, no bias
	HMC5883L_BIAS_POSITIVE,   //!< Positive bias configuration all axes, used for self test (see datasheet)
	HMC5883L_BIAS_NEGATIVE    //!< Negative bias configuration all axes, used for self test (see datasheet)
} hmc5883l_bias_t;

typedef enum {
	HMC5883L_GAIN_1370 = 0, //!< 0.73 mG/LSb, range -0.88..+0.88 G
	HMC5883L_GAIN_1090,     //!< 0.92 mG/LSb, range -1.3..+1.3 G, default
	HMC5883L_GAIN_820,      //!< 1.22 mG/LSb, range -1.9..+1.9 G
	HMC5883L_GAIN_660,      //!< 1.52 mG/LSb, range -2.5..+2.5 G
	HMC5883L_GAIN_440,      //!< 2.27 mG/LSb, range -4.0..+4.0 G
	HMC5883L_GAIN_390,      //!< 2.56 mG/LSb, range -4.7..+4.7 G
	HMC5883L_GAIN_330,      //!< 3.03 mG/LSb, range -5.6..+5.6 G
	HMC5883L_GAIN_230       //!< 4.35 mG/LSb, range -8.1..+8.1 G
} hmc5883l_gain_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} hmc5883l_raw_data_t;

typedef struct {
	float x;
	float y;
	float z;
} hmc5883l_data_t;

esp_err_t drv_hmc5883_init();

esp_err_t drv_hmc5883_ping();


esp_err_t hmc5883l_init_desc(hmc5883l_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

esp_err_t hmc5883l_free_desc(hmc5883l_dev_t *dev);

esp_err_t hmc5883l_init(hmc5883l_dev_t *dev);

esp_err_t hmc5883l_get_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t *val);

esp_err_t hmc5883l_set_opmode(hmc5883l_dev_t *dev, hmc5883l_opmode_t mode);

esp_err_t hmc5883l_get_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t *val);

esp_err_t hmc5883l_set_samples_averaged(hmc5883l_dev_t *dev, hmc5883l_samples_averaged_t samples);

esp_err_t hmc5883l_get_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t *val);

esp_err_t hmc5883l_set_data_rate(hmc5883l_dev_t *dev, hmc5883l_data_rate_t rate);

esp_err_t hmc5883l_get_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t *val);

esp_err_t hmc5883l_set_bias(hmc5883l_dev_t *dev, hmc5883l_bias_t bias);

esp_err_t hmc5883l_get_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t *val);

esp_err_t hmc5883l_set_gain(hmc5883l_dev_t *dev, hmc5883l_gain_t gain);

esp_err_t hmc5883l_data_is_ready(hmc5883l_dev_t *dev, bool *val);

esp_err_t hmc5883l_data_is_locked(hmc5883l_dev_t *dev, bool *val);

esp_err_t hmc5883l_get_raw_data(hmc5883l_dev_t *dev, hmc5883l_raw_data_t *data);

esp_err_t hmc5883l_raw_to_mg(const hmc5883l_dev_t *dev, const hmc5883l_raw_data_t *raw, hmc5883l_data_t *mg);

esp_err_t hmc5883l_get_data(hmc5883l_dev_t *dev, hmc5883l_data_t *data);

#endif // DRV_HMC5883_H
