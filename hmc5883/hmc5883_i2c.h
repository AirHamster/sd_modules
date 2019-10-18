/*
 * hmc5883_i2c.h
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */



#ifndef SD_MODULES_hmc5883_hmc5883_I2C_H_
#define SD_MODULES_hmc5883_hmc5883_I2C_H_

#include "stdint.h"
#include "ch.h"
#include "hal.h"
//#include "hmc5883.h"

#define HMC5883_I2C_ADDR				0x1E
#define HMC5883_CONFIG_REG_A_ADDR		0x00
#define HMC5883_CONFIG_REG_B_ADDR		0x01
#define HMC5883_MODE_REG_ADDR			0x02
#define HMC5883_DATA_X_MSB_REG_ADDR		0x03
#define HMC5883_DATA_X_LSB_REG_ADDR		0x04
#define HMC5883_DATA_Z_MSB_REG_ADDR		0x05
#define HMC5883_DATA_Z_LSB_REG_ADDR		0x06
#define HMC5883_DATA_Y_MSB_REG_ADDR		0x07
#define HMC5883_DATA_Y_LSB_REG_ADDR		0x08
#define HMC5883_STATUS_REG_ADDR			0x09
#define HMC5883_IDENT_A_REG_ADDR		0x0A
#define HMC5883_IDENT_B__REG_ADDR		0x0B
#define HMC5883_IDENT_C__REG_ADDR		0x0C

#define PI 3.1415f
typedef struct {
	float yaw;
	float x;
	float y;
	float z;
	uint8_t config_reg_a;
	uint8_t config_reg_b;
	uint8_t mode_reg;
	uint8_t status_reg;
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
} hmc5883_t;
struct HMC5883L_result {
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t valid;
};
typedef struct HMC5883L_result HMC5883L_result;
#define I2C_BUFFER_LEN 128
void start_hmc5883_module(void);
void hmc5883_calibration(float * dest1, float * dest2);
uint8_t read_data_HMC5883L(HMC5883L_result * dest);
int8_t hmc5883_full_init(void);
uint8_t hmc5883_read_data(hmc5883_t *hmc);
int8_t hmc5883_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len);
int8_t hmc5883_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len);
int8_t hmc5883_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t hmc5883_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void hmc5883_calculate(void);
void hmc5883_delay_ms(uint16_t msec);
#endif /* SD_MODULES_hmc5883_hmc5883_I2C_H_ */
