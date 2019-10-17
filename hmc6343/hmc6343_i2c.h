/*
 * hmc6343_i2c.h
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */



#ifndef SD_MODULES_HMC6343_HMC6343_I2C_H_
#define SD_MODULES_HMC6343_HMC6343_I2C_H_

#include "stdint.h"
#include "ch.h"
#include "hal.h"
//#include "hmc6343.h"

#define HMC6343_I2C_ADDR				0x19
#define HMC6343_POST_HEADING_DATA       0x50
#define HMC6343_READ_EEPROM_CMD			0xE1
#define HMC6343_WRITE_EEPROM_CMD			0xF1

#define HMC6343_EEPROM_START			0x00

#define HMC6343_CONFIG_REG_A_ADDR		0x00
#define HMC6343_CONFIG_REG_B_ADDR		0x01
#define HMC6343_MODE_REG_ADDR			0x02
#define HMC6343_DATA_X_MSB_REG_ADDR		0x03
#define HMC6343_DATA_X_LSB_REG_ADDR		0x04
#define HMC6343_DATA_Z_MSB_REG_ADDR		0x05
#define HMC6343_DATA_Z_LSB_REG_ADDR		0x06
#define HMC6343_DATA_Y_MSB_REG_ADDR		0x07
#define HMC6343_DATA_Y_LSB_REG_ADDR		0x08
#define HMC6343_STATUS_REG_ADDR			0x09
#define HMC6343_IDENT_A_REG_ADDR		0x0A
#define HMC6343_IDENT_B__REG_ADDR		0x0B
#define HMC6343_IDENT_C__REG_ADDR		0x0C

#define PI 3.1415f
typedef struct {
	float yaw;
	float roll;
	float pitch;
    int16_t yaw16;
    int16_t pitch16;
    int16_t roll16;
} hmc6343_t;

struct HMC6343L_result {
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t valid;
};
typedef struct HMC6343L_result HMC6343L_result;
#define I2C_BUFFER_LEN 128
void start_hmc6343_module(void);
void hmc6343_calibration(float * dest1, float * dest2);
uint8_t read_data_HMC6343L(HMC6343L_result * dest);
int8_t hmc6343_full_init(void);
uint8_t hmc6343_read_data(hmc6343_t *hmc);
int8_t hmc6343_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len);
int8_t hmc6343_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len);
int8_t hmc6343_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t hmc6343_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void hmc6343_calculate(void);
void hmc6343_delay_ms(uint16_t msec);
#endif /* SD_MODULES_hmc6343_hmc6343_I2C_H_ */
