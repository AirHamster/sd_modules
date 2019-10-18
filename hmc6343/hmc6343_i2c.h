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

#define HMC6343_POST_ACCEL_CMD			0x40
#define HMC6343_POST_MAG_CMD			0x45
#define HMC6343_POST_HEADING_DATA       0x50
#define HMC6343_POST_TILT_CMD			0x55
#define HMC6343_READ_OP_MODE_CMD		0x65
#define HMC6343_ENTER_USER_CALIBRATE_CMD	0x71
#define HMC6343_LEVEL_ORIENT_CMD		0x72
#define HMC6343_UPRIGHT_SIDEWAYS_ORIENT_CMD		0x73
#define HMC6343_UPRIGHT_FLAT_FRONT_ORIENT_CMD	0x74
#define HMC6343_ENTER_RUN_MODE_CMD		0x75
#define HMC6343_ENTER_STANDBY_MODE_CMD	0x76
#define HMC6343_EXIT_USER_CALIBRATE_CMD	0x7E
#define HMC6343_RESET_CMD				0x82
#define HMC6343_ENTER_SLEEP_MODE_CMD	0x83
#define HMC6343_EXIT_SLEEP_MODE_CMD		0x84
#define HMC6343_READ_EEPROM_CMD			0xE1
#define HMC6343_WRITE_EEPROM_CMD		0xF1

#define HMC6343_EEPROM_SLAVE_ADDR		0x00
#define HMC6343_EEPROM_SW_VER			0x02
#define HMC6343_EEPROM_OP_MODE1			0x04
#define HMC6343_EEPROM_OP_MODE2			0x05
#define HMC6343_EEPROM_SERIAL_NUM_LSB	0x06
#define HMC6343_EEPROM_SERIAL_NUM_MSB	0x07
#define HMC6343_EEPROM_DATE_CODE_YY		0x08
#define HMC6343_EEPROM_DATE_CODE_WW		0x09
#define HMC6343_EEPROM_DEVIATION_LSB	0x0A
#define HMC6343_EEPROM_DEVIATION_MSB	0x0B
#define HMC6343_EEPROM_VARIATION_LSB	0x0C
#define HMC6343_EEPROM_VARIATION_MSB	0x0D
#define HMC6343_EEPROM_X_OFFSET_LSB		0x0E
#define HMC6343_EEPROM_X_OFFSET_MSB		0x0F
#define HMC6343_EEPROM_Y_OFFSET_LSB		0x10
#define HMC6343_EEPROM_Y_OFFSET_MSB		0x11
#define HMC6343_EEPROM_Z_OFFSET_LSB		0x12
#define HMC6343_EEPROM_Z_OFFSET_MSB		0x13
#define HMC6343_EEPROM_FILTER_LSB		0x14
#define HMC6343_EEPROM_FILTER_MSB		0x15

#define PI 3.1415f
typedef struct {
	float yaw;
	float roll;
	float pitch;
    int16_t yaw16;
    int16_t pitch16;
    int16_t roll16;
    int16_t x_offset;
    int16_t y_offset;
    int16_t z_offset;
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
void hmc6343_start_calibration(hmc6343_t *hmc);
void hmc6343_stop_calibration(hmc6343_t *hmc);
void hmc6343_read_calib_data(hmc6343_t *hmc);
void hmc6343_delay_ms(uint16_t msec);
#endif /* SD_MODULES_hmc6343_hmc6343_I2C_H_ */
