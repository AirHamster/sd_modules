#ifndef EEPROM_H
#define EEPROM_H

#include <stdlib.h>
#include "stdint.h"
#include "config.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#ifdef SD_MODULE_TRAINER
#define EEPROM_ADDRESS 0x57
#endif

#ifdef SENSOR_BOX
#define EEPROM_ADDRESS	0x50
#endif

//int16_t
#define EEPROM_INFO_MEMORY_START		0x0000	//32 bytes for info
#define EEPROM_FLAGS_MEMORY_START		0x001F	//32 bytes for flags
#define EEPROM_CALIB_MEMORY_START		0x003F	//all other for calibrations

//int16_t
#define EEPROM_HW_VER_ADDR 				EEPROM_INFO_MEMORY_START
#define EEPROM_SW_VER_ADDR 				EEPROM_INFO_MEMORY_START + 1

//int8_t
#define EEPROM_STATIC_CALIB_FLAG_ADDR	EEPROM_FLAGS_MEMORY_START
#define EEPROM_RUDDER_CALIB_FLAG_ADDR	EEPROM_FLAGS_MEMORY_START + 1


//int16_t
#define EEPROM_MAGN_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START
#define EEPROM_MAGN_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 2
#define EEPROM_MAGN_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 4
#define EEPROM_MAGN_RADIUS_ADDR			EEPROM_CALIB_MEMORY_START + 6

//int16_t
#define EEPROM_GYRO_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 8
#define EEPROM_GYRO_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 10
#define EEPROM_GYRO_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 12

//int16_t
#define EEPROM_ACCEL_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 14
#define EEPROM_ACCEL_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 16
#define EEPROM_ACCEL_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 18
#define EEPROM_ACCEL_RADIUS_ADDR		EEPROM_CALIB_MEMORY_START + 20

/*
 *
 * three int16_t values here
 *
 */

//float value
#define EEPROM_LAG_CALIB_NUMBER			EEPROM_CALIB_MEMORY_START + 28

//float values
#define EEPROM_MATH_COMPASS_CORRECTION			EEPROM_CALIB_MEMORY_START + 32
#define EEPROM_MATH_HSP_CORRECTION				EEPROM_CALIB_MEMORY_START + 36
#define EEPROM_MATH_HEEL_CORRECTION				EEPROM_CALIB_MEMORY_START + 40
#define EEPROM_MATH_DECLANATION_CORRECTION		EEPROM_CALIB_MEMORY_START + 44
#define EEPROM_MATH_PITCH_CORRECTION			EEPROM_CALIB_MEMORY_START + 48
#define EEPROM_MATH_RUDDER_CORRECTION			EEPROM_CALIB_MEMORY_START + 52
#define EEPROM_MATH_WIND_CORRECTION				EEPROM_CALIB_MEMORY_START + 56

//uint8_t values
#define EEPROM_MATH_WINSIZE1_CORRECTION			EEPROM_CALIB_MEMORY_START + 60
#define EEPROM_MATH_WINSIZE2_CORRECTION			EEPROM_CALIB_MEMORY_START + 61
#define EEPROM_MATH_WINSIZE3_CORRECTION			EEPROM_CALIB_MEMORY_START + 62

//float
#define EEPROM_RUDDER_CALIB_NATIVE_LEFT			EEPROM_CALIB_MEMORY_START + 66 	//costil adress
#define EEPROM_RUDDER_CALIB_NATIVE_RIGHT		EEPROM_CALIB_MEMORY_START + 70
#define EEPROM_RUDDER_CALIB_NATIVE_CENTER		EEPROM_CALIB_MEMORY_START + 74

#define EEPROM_RUDDER_CALIB_DEGREES_LEFT		EEPROM_CALIB_MEMORY_START + 78
#define EEPROM_RUDDER_CALIB_DEGREES_RIGHT		EEPROM_CALIB_MEMORY_START + 82
#define EEPROM_RUDDER_CALIB_DEGREES_CENTER		EEPROM_CALIB_MEMORY_START + 86

#define BNO055_ADDRESS	0x28

int8_t eeprom_write(uint16_t byte_addr, const uint8_t *txbuf, uint8_t txbytes);
int8_t eeprom_read(uint16_t byte_addr, uint8_t *rxbuf, uint8_t rxbytes);
void eeprom_check_i2c_bus(void);
void eeprom_read_hw_version(void);
void eeprom_write_hw_version(void);
void bno055_read_id(void);
void start_eeprom_module(void);
#endif
