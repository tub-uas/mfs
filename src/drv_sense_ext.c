#include "drv_sense_ext.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "util.h"

/* Please see the Datasheet: http://www.ti.com/lit/ds/symlink/ina219.pdf or
 * this very helpful Site: http://cdwilson.us/articles/understanding-the-INA219/
 * for more Information.
 */

/* General I2C Settings */
#define SENSE_EXT_I2C_SCL_PIN    32
#define SENSE_EXT_I2C_SDA_PIN    33
#define SENSE_EXT_I2C_PORT_NUM   0	// Using Port 0
#define SENSE_EXT_I2C_FREQ_HZ    100000	// I2C Frequency 100kHz
#define SENSE_EXT_I2C_TIMEOUT_MS 1000	// Timeout after 1000ms

/* Register Addresses inside INA219 */
#define SENSE_EXT_REG_CONFIG     0
#define SENSE_EXT_REG_SHUNT_U    1
#define SENSE_EXT_REG_BUS_U      2
#define SENSE_EXT_REG_POW        3
#define SENSE_EXT_REG_CUR        4
#define SENSE_EXT_REG_CAL        5

/* Bits inside INA219 Configuration Regsiter */
#define SENSE_EXT_BIT_RST        15
#define SENSE_EXT_BIT_BRNG       13
#define SENSE_EXT_BIT_PG0        11
#define SENSE_EXT_BIT_BADC0      7
#define SENSE_EXT_BIT_SADC0      3
#define SENSE_EXT_BIT_MODE       0

/*           -- Main Sensor Configuration --
 * The Power Board is using two 0.001 Ohm / 3 Watt Resistors in parallel to
 * make "one" large 0.000500 Ohm / 6 Watt Resistor. It is designed to handle
 * 30 Volts and 100 Amp.
 * At 100 Amp we have a Voltage Drop of 0.050 Volt and a Power Dissipation
 * of 5 Watt.
 * To reduce Noise, we use the integrated Multisampling Function. We also use
 * the full 12 Bit Range of the INA219.
 * We therefore, in the Configuration Register, set:
 * - Busrange (BRNG) to 1, which allows up to 32 Volt.
 * - Amplifier (PG) Fullscale Range to 1, which allows up to 0.080 Volt of Voltage Drop
 * - Resolution / Sampling (BADC / SADC) to 12, which averages 16 Samples with 12 Bit (takes 8.51ms)
 * - Mode (MODE) to 7, which enables continuous Sampling of both Shunt and Bus
 *
 * Since we have a theoretical Maximum Resolution (thanks to averaging) of 15 Bit
 * and an expected Maximum Current of 100 Amp, our LSB is:
 *   current_lsb =  100 / 15^2 = 0.003051757.
 * For simplicity, we will choose a Value of:
 *   current_lsb = 0.004
 * We can than plug that into the following equation and calculate the value for
 * the Calibration Register:
 *   cal = trunc(0.04096 / (current_lsb * r_shunt))
 *       = trunc(0.04096 / (0.004 * 0.0005))
 *       = 20480
 *	This value will be written to the Calibration Register.
 *
 *            -- Power and Sys Sensor Configuration --
 * The Configuration of the Power and Sys Sensors is identical and very similar
 * to the configuration of the Main Sensor.
 * The Power and Sys Sensors are using 0.005 Ohm 1 Watt Resistors. They are
 * desigend to handle up to 15 Volts and 10 Amp.
 * At 10 Amp we have a Voltage Drop of 0.05 Volt and a Power Dissipation
 * of 0.5 Watt.
 * To reduce Noise, we use the integrated Multisampling Function. We also use
 * the full 12 Bit Range of the INA219.
 * We therefore, in the Configuration Register, set:
 * - Busrange (BRNG) to 0, which allows up to 16 Volt.
 * - Amplifier (PG) Fullscale Range to 1, which allows up to 0.080 Volt of Voltage Drop
 * - Resolution / Sampling (BADC / SADC) to 12, which averages 16 Samples with 12 Bit (takes 8.51ms)
 * - Mode (MODE) to 7, which enables continuous Sampling of both Shunt and Bus
 *
 * Since we have a theoretical Maximum Resolution (thanks to averaging) of 15 Bit
 * and an expected Maximum Current of 10 Amp, our LSB is:
 *   current_lsb =  10 / 15^2 = 0.0003051757.
 * For simplicity, we will choose a Value of:
 *   current_lsb = 0.0004
 * We can than Plug that into the following equation and calculate the value for
 * the Calibration Register:
 *   cal = trunc(0.04096 / (current_lsb * r_shunt))
 *       = trunc(0.04096 / (0.0004 * 0.005))
 *       = 20480
 *	This value will be written to the Calibration Register.
 */

/* Main Sensor */
#ifdef SENSE_EXT_MAIN_ADDR
#define SENSE_EXT_MAIN_LSB  0.004f
// #define SENSE_EXT_MAIN_CAL	  20480
#define SENSE_EXT_MAIN_CAL  20100 // Calibrated value to board set 1
#endif

/* Power Sensor */
#ifdef SENSE_EXT_PWR_ADDR
#define SENSE_EXT_PWR_LSB   0.0004f
// #define SENSE_EXT_PWR_CAL   20480
#define SENSE_EXT_PWR_CAL   18600 // Calibrated value to board set 1
#endif

/* Sys Sensor */
#ifdef SENSE_EXT_SYS_ADDR
#define SENSE_EXT_SYS_LSB    0.0004f
// #define SENSE_EXT_SYS_CAL    20480
#define SENSE_EXT_SYS_CAL    19500 // Calibrated value to board set 1
#endif


esp_err_t drv_sense_ext_read_reg(uint8_t addr, uint8_t reg, uint16_t *val);

esp_err_t drv_sense_ext_write_reg(uint8_t addr, uint8_t reg, uint16_t val);

esp_err_t drv_sense_ext_read_shunt_volt(uint8_t addr, float *shunt_volt);


esp_err_t drv_sense_ext_init() {

	ESP_LOGI(__FILE__, "Initalizing external I2C Voltage and Current Sensors");

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.scl_io_num = SENSE_EXT_I2C_SCL_PIN;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.sda_io_num = SENSE_EXT_I2C_SDA_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = SENSE_EXT_I2C_FREQ_HZ;
	esp_err_t ret_i2c = i2c_param_config(SENSE_EXT_I2C_PORT_NUM, &conf);

	/* Start the I2C Driver */
	ret_i2c |= i2c_driver_install(SENSE_EXT_I2C_PORT_NUM, conf.mode, 0, 0, 0);
	if (ret_i2c != ESP_OK)
		ESP_LOGE(__FILE__, "Cant start I2C Driver");

	/* Main Sensor */
	#ifdef SENSE_EXT_MAIN_ADDR

	esp_err_t ret_main = drv_sense_ext_write_reg(SENSE_EXT_MAIN_ADDR,
	                                             SENSE_EXT_REG_CONFIG,
	                                        1 << SENSE_EXT_BIT_RST);

	ret_main |= drv_sense_ext_write_reg(SENSE_EXT_MAIN_ADDR,
	                                    SENSE_EXT_REG_CONFIG,
	                              (1 << SENSE_EXT_BIT_BRNG)
	                            | (1 << SENSE_EXT_BIT_PG0)
	                           | (12 << SENSE_EXT_BIT_SADC0)
	                           | (12 << SENSE_EXT_BIT_BADC0)
	                            | (7 << SENSE_EXT_BIT_MODE));

	/* Set the correct Calibration */
	ret_main |= drv_sense_ext_write_reg(SENSE_EXT_MAIN_ADDR,
	                                    SENSE_EXT_REG_CAL,
	                                    SENSE_EXT_MAIN_CAL);

	if (ret_main != ESP_OK)
		ESP_LOGE(__FILE__, "Cant initalize external Main Sensor");

	#else
	ESP_LOGW(__FILE__, "Not initializing external Main Sensor");
	#endif

	/* Power Sensor */
	#ifdef SENSE_EXT_PWR_ADDR
	esp_err_t ret_power = drv_sense_ext_write_reg(SENSE_EXT_PWR_ADDR,
	                                              SENSE_EXT_REG_CONFIG,
	                                         1 << SENSE_EXT_BIT_RST);

	ret_power |= drv_sense_ext_write_reg(SENSE_EXT_PWR_ADDR,
	                                     SENSE_EXT_REG_CONFIG,
	                               (1 << SENSE_EXT_BIT_BRNG)
	                             | (1 << SENSE_EXT_BIT_PG0)
	                            | (12 << SENSE_EXT_BIT_SADC0)
	                            | (12 << SENSE_EXT_BIT_BADC0)
	                             | (7 << SENSE_EXT_BIT_MODE));

	ret_power |= drv_sense_ext_write_reg(SENSE_EXT_PWR_ADDR,
	                                     SENSE_EXT_REG_CAL,
	                                     SENSE_EXT_PWR_CAL);

	if (ret_power != ESP_OK)
		ESP_LOGE(__FILE__, "Cant initalize external Power Sensor");

	#else
	ESP_LOGW(__FILE__, "Not initializing external Power Sensor");
	#endif

	/* Sys Sensor */
	#ifdef SENSE_EXT_SYS_ADDR
	esp_err_t ret_sys = drv_sense_ext_write_reg(SENSE_EXT_SYS_ADDR,
	                                           SENSE_EXT_REG_CONFIG,
	                                      1 << SENSE_EXT_BIT_RST);

	ret_sys |= drv_sense_ext_write_reg(SENSE_EXT_SYS_ADDR,
	                                  SENSE_EXT_REG_CONFIG,
	                            (1 << SENSE_EXT_BIT_BRNG)
	                          | (1 << SENSE_EXT_BIT_PG0)
	                         | (12 << SENSE_EXT_BIT_SADC0)
	                         | (12 << SENSE_EXT_BIT_BADC0)
	                          | (7 << SENSE_EXT_BIT_MODE));

	ret_sys |= drv_sense_ext_write_reg(SENSE_EXT_SYS_ADDR,
	                                  SENSE_EXT_REG_CAL,
	                                  SENSE_EXT_SYS_CAL);

	if (ret_sys != ESP_OK)
		ESP_LOGE(__FILE__, "Cant initalize external SYS Sensor");

	#else
	ESP_LOGW(__FILE__, "Not initializing external SYS Sensor");
	#endif

	return ret_i2c
	     #ifdef SENSE_EXT_MAIN_ADDR
	     | ret_main
	     #endif
	     #ifdef SENSE_EXT_PWR_ADDR
	     | ret_power
	     #endif
	     #ifdef SENSE_EXT_SYS_ADDR
	     | ret_sys
	     #endif
	     ;
}

esp_err_t drv_sense_ext_read_reg(uint8_t addr, uint8_t reg, uint16_t *val) {

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(cmd, (uint8_t*)val, 2, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

	esp_err_t ret = i2c_master_cmd_begin(SENSE_EXT_I2C_PORT_NUM,
	                                     cmd,
	                                     SENSE_EXT_I2C_TIMEOUT_MS / portTICK_RATE_MS);

	if (ret != ESP_OK)
		ESP_LOGW(__FILE__, "Cant read I2C Register ");

	i2c_cmd_link_delete(cmd);

	*val = (*val>>8) | (*val<<8);

	return ret;
}

esp_err_t drv_sense_ext_write_reg(uint8_t addr, uint8_t reg, uint16_t val) {

	val = (val>>8) | (val<<8);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write(cmd, (uint8_t*)&val, 2, true);
	i2c_master_stop(cmd);

	esp_err_t ret = i2c_master_cmd_begin(SENSE_EXT_I2C_PORT_NUM,
	                                     cmd,
	                                     SENSE_EXT_I2C_TIMEOUT_MS / portTICK_RATE_MS);

	if (ret != ESP_OK)
		ESP_LOGW(__FILE__, "Cant write I2C Register");

	i2c_cmd_link_delete(cmd);

	return ret;
}

esp_err_t drv_sense_ext_read_volt(uint8_t addr, float *volt) {

	uint16_t volt_raw = 0;
	esp_err_t ret = drv_sense_ext_read_reg(addr, SENSE_EXT_REG_BUS_U, (uint16_t*)&volt_raw);

	*volt = (volt_raw >> 3) * 0.004;

	if (ret != ESP_OK)
		ESP_LOGE(__FILE__, "Cant read Voltage from external Sensor, Addr: 0x%02x", addr);

	return ret;
}

esp_err_t drv_sense_ext_read_shunt_volt(uint8_t addr, float *shunt_volt) {

	int16_t shunt_volt_raw = 0;
	esp_err_t ret = drv_sense_ext_read_reg(addr, SENSE_EXT_REG_SHUNT_U, (uint16_t*)&shunt_volt_raw);

	*shunt_volt = shunt_volt_raw / 100000.0;

	if (ret != ESP_OK)
		ESP_LOGE(__FILE__, "Cant read Shunt Voltage from external Sensor, Addr: 0x%02x", addr);

	return ret;
}

esp_err_t drv_sense_ext_read_curr(uint8_t addr, float *curr) {

	int16_t curr_raw = 0;
	esp_err_t ret = drv_sense_ext_read_reg(addr, SENSE_EXT_REG_CUR, (uint16_t*)&curr_raw);

	switch (addr) {
		#ifdef SENSE_EXT_MAIN_ADDR
		case SENSE_EXT_MAIN_ADDR:
			*curr = curr_raw * SENSE_EXT_MAIN_LSB;
			break;
		#endif
		#ifdef SENSE_EXT_PWR_ADDR
		case SENSE_EXT_PWR_ADDR:
			*curr = curr_raw * SENSE_EXT_PWR_LSB;
			break;
		#endif
		#ifdef SENSE_EXT_SYS_ADDR
		case SENSE_EXT_SYS_ADDR:
			*curr = curr_raw * SENSE_EXT_SYS_LSB;
			break;
		#endif
		default:
			ret |= ESP_ERR_INVALID_ARG;
			break;
	}

	if (ret != ESP_OK)
		ESP_LOGE(__FILE__, "Cant read Current from external Sensor, Addr: 0x%02x", addr);

	return ret;
}

esp_err_t drv_sense_ext_read_pow(uint8_t addr, float *pow) {

	int16_t pow_raw = 0;
	esp_err_t ret = drv_sense_ext_read_reg(addr, SENSE_EXT_REG_POW, (uint16_t*)&pow_raw);

	switch (addr) {
		#ifdef SENSE_EXT_MAIN_ADDR
		case SENSE_EXT_MAIN_ADDR:
			*pow = pow_raw * 20.0 * SENSE_EXT_MAIN_LSB;
			break;
		#endif
		#ifdef SENSE_EXT_PWR_ADDR
		case SENSE_EXT_PWR_ADDR:
			*pow = pow_raw * 20.0 * SENSE_EXT_PWR_LSB;
			break;
		#endif
		#ifdef SENSE_EXT_SYS_ADDR
		case SENSE_EXT_SYS_ADDR:
			*pow = pow_raw * 20.0 * SENSE_EXT_SYS_LSB;
			break;
		#endif
		default:
			ret |= ESP_ERR_INVALID_ARG;
			break;
	}

	if (ret != ESP_OK)
		ESP_LOGE(__FILE__, "Cant read Power from external Sensor, Addr: 0x%02x", addr);

	return ret;
}

void drv_sense_ext_test() {

	ESP_LOGI(__FILE__, "External I2C Voltage and Current Sensor Test");

	while (1) {

		// uint8_t sensor_addr = SENSE_EXT_MAIN_ADDR;
		// uint8_t sensor_addr = SENSE_EXT_PWR_ADDR;
		uint8_t sensor_addr = SENSE_EXT_SYS_ADDR;

		float volt = 0.0;
		float curr = 0.0;
		float pow = 0.0;

		ESP_ERROR_CHECK(drv_sense_ext_read_volt(sensor_addr, &volt));
		ESP_ERROR_CHECK(drv_sense_ext_read_curr(sensor_addr, &curr));
		ESP_ERROR_CHECK(drv_sense_ext_read_pow(sensor_addr, &pow));

		printf("Addr: 0x%02x    ", sensor_addr);
		printf("Voltage: %10f    ", volt);
		printf("Current: %10f    ", curr);
		printf("Power: %10f    \n", pow);

		delay_ms(100);
	}
}
