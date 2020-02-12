#ifndef EEPROM_H
#define EEPROM_H

#include <stdlib.h>
#include <stddef.h>
#include "stdint.h"
#include "config.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

//#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#ifdef SD_MODULE_TRAINER
#define EEPROM_ADDRESS 0x57
#endif

#ifdef SD_SENSOR_BOX
#define EEPROM_ADDRESS	0x50
#endif

#define EEPROM_INFO_MEMORY_SIZE 	32
#define EEPROM_FLAGS_MEMORY_SIZE	32
#define EEPROM_MAGN_MEMORY_SIZE		64
#define EEPROM_MATH_MEMORY_SIZE		64
#define EEPROM_LAG_MEMORY_SIZE		64
#define EEPROM_RUDDER_MEMORY_SIZE	64
#define EEPROM_TENSO_MEMORY_SIZE	64
#define EEPROM_XBEE_ADDRESES_MEMORY_SIZE	128
#define EEPROM_BLE_ADDRESES_MEMORY_SIZE		128


typedef struct{
	int8_t HW_VER;
	int8_t FW_VER;
}eeprom_info_memory_t;

typedef struct{
	uint8_t STATIC_MAGN_CALIB_FLAG;
	uint8_t RUDDER_CALIB_FLAG;
}eeprom_flags_memory_t;

typedef struct{
	int16_t MAGN_X_OFFSET;
	int16_t MAGN_Y_OFFSET;
	int16_t MAGN_Z_OFFSET;
	int16_t MAGN_RADIUS;
}eeprom_magn_calib_memory_t;

typedef struct{
	//float values
	float MATH_COMPASS_CORRECTION;
	float MATH_HSP_CORRECTION;
	float MATH_HEEL_CORRECTION;
	float MATH_DECLANATION_CORRECTION;
	float MATH_PITCH_CORRECTION;
	float MATH_RUDDER_CORRECTION;
	float MATH_WIND_CORRECTION;

	//uint8_t values
	uint8_t MATH_WINSIZE1_CORRECTION;
	uint8_t MATH_WINSIZE2_CORRECTION;
	uint8_t MATH_WINSIZE3_CORRECTION;
}eeprom_math_corrections_t;

typedef struct{
	float LAG_CALIB;
}eeprom_lag_calib_memory_t;

typedef struct{
	//float
	float RUDDER_CALIB_NATIVE_LEFT;
	float RUDDER_CALIB_NATIVE_RIGHT;
	float RUDDER_CALIB_NATIVE_CENTER;

	float RUDDER_CALIB_DEGREES_LEFT;
	float RUDDER_CALIB_DEGREES_RIGHT;
	float RUDDER_CALIB_DEGREES_CENTER;
}eeprom_rudder_calib_memory_t;

typedef struct{
	//uint16_t
	uint16_t TENSO_POINT_ONE_NATIVE;
	uint16_t TENSO_POINT_TWO_NATIVE;
	//float
	float TENSO_POINT_ONE_KILOGRAMS;
	float TENSO_POINT_TWO_KILOGRAMS;
	float TENSO_COEF_KILOGRAMS;
	float TENSO_COEF_NEWTONS;
	float TENSO_OFFSET;
}eeprom_tenso_calib_memory_t;

typedef struct{
	//8 bytes arrays
	uint8_t XBEE_TRAINER_ADDR[8];
	uint8_t XBEE_SPORTSMAN_1_ADDR[8];
	uint8_t XBEE_SPORTSMAN_2_ADDR[8];
	uint8_t XBEE_SPORTSMAN_3_ADDR[8];
	uint8_t XBEE_SPORTSMAN_4_ADDR[8];
	uint8_t XBEE_SPORTSMAN_5_ADDR[8];
	uint8_t XBEE_SPORTSMAN_6_ADDR[8];
	uint8_t XBEE_SPORTSMAN_7_ADDR[8];
	uint8_t XBEE_SPORTSMAN_8_ADDR[8];
	uint8_t XBEE_SPORTSMAN_9_ADDR[8];
	uint8_t XBEE_SPORTSMAN_10_ADDR[8];

	uint8_t XBEE_BOUY_1_ADDR[8];
	uint8_t XBEE_BOUY_2_ADDR[8];
	uint8_t XBEE_BOUY_3_ADDR[8];
	uint8_t XBEE_BOUY_4_ADDR[8];

}eeprom_xbee_addresses_t;

typedef struct{
	uint8_t BLE_DEV_1_ADDR[12];
	uint8_t BLE_DEV_2_ADDR[12];
	uint8_t BLE_DEV_3_ADDR[12];
	uint8_t BLE_DEV_4_ADDR[12];
	uint8_t BLE_DEV_5_ADDR[12];
	uint8_t BLE_DEV_6_ADDR[12];
	uint8_t BLE_DEV_7_ADDR[12];
	uint8_t BLE_DEV_8_ADDR[12];
}eeprom_ble_addresses_t;


typedef struct {
	union {
		uint8_t info_allocator[EEPROM_INFO_MEMORY_SIZE];
		eeprom_info_memory_t INFO_MEMORY;
	};
	union {
		uint8_t flags_allocator[EEPROM_FLAGS_MEMORY_SIZE];
		eeprom_flags_memory_t FLAGS_MEMORY;
	};
	union {
		uint8_t magn_allocator[EEPROM_MAGN_MEMORY_SIZE];
		eeprom_magn_calib_memory_t MAGN_MEMORY;
	};
	union {
		uint8_t math_allocator[EEPROM_MATH_MEMORY_SIZE];
		eeprom_math_corrections_t MATH_MEMORY;
	};
	union {
		uint8_t lag_allocator[EEPROM_LAG_MEMORY_SIZE];
		eeprom_lag_calib_memory_t LAG_MEMORY;
	};
	union {
		uint8_t rdr_allocator[EEPROM_RUDDER_MEMORY_SIZE];
		eeprom_rudder_calib_memory_t RUDDER_MEMORY;
	};
	union {
		uint8_t tenso_allocator[EEPROM_TENSO_MEMORY_SIZE];
		eeprom_tenso_calib_memory_t TENSO_MEMORY;
	};
	union {
		uint8_t xbee_allocator[EEPROM_XBEE_ADDRESES_MEMORY_SIZE];
		eeprom_xbee_addresses_t XBEE_MEMORY;
	};
	union {
		uint8_t ble_allocator[EEPROM_BLE_ADDRESES_MEMORY_SIZE];
		eeprom_ble_addresses_t BLE_MEMORY;
	};
} eeprom_map_t;

#define INFO_STRUCT_SIZE sizeof(eeprom_info_memory_t)

//#if (sizeof(eeprom_info_memory_t) > EEPROM_INFO_MEMORY_SIZE)
//	#error ***EEPROM memory map error! Varables structs looks not properly alocated!***
//#endif

/*
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
#define EEPROM_MAGN_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 4
#define EEPROM_MAGN_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 8
#define EEPROM_MAGN_RADIUS_ADDR			EEPROM_CALIB_MEMORY_START + 12

//int16_t
//#define EEPROM_GYRO_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 8
//#define EEPROM_GYRO_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 10
//#define EEPROM_GYRO_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 12

//int16_t
//#define EEPROM_ACCEL_X_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 14
//#define EEPROM_ACCEL_Y_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 16
//#define EEPROM_ACCEL_Z_OFFSET_ADDR		EEPROM_CALIB_MEMORY_START + 18
//#define EEPROM_ACCEL_RADIUS_ADDR		EEPROM_CALIB_MEMORY_START + 20


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

//uint16_t
#define EEPROM_TENSO_POINT_ONE_NATIVE			EEPROM_CALIB_MEMORY_START + 90
#define EEPROM_TENSO_POINT_TWO_NATIVE			EEPROM_CALIB_MEMORY_START + 92
//float
#define EEPROM_TENSO_POINT_ONE_KILOGRAMS			EEPROM_CALIB_MEMORY_START + 94
#define EEPROM_TENSO_POINT_TWO_KILOGRAMS			EEPROM_CALIB_MEMORY_START + 98
#define EEPROM_TENSO_COEF_KILOGRAMS				EEPROM_CALIB_MEMORY_START + 102
#define EEPROM_TENSO_COEF_NEWTONS				EEPROM_CALIB_MEMORY_START + 106
#define EEPROM_TENSO_OFFSET						EEPROM_CALIB_MEMORY_START + 110

//8 bytes arrays
#define EEPROM_XBEE_TRAINER_ADDR				EEPROM_CALIB_MEMORY_START + 114
#define EEPROM_XBEE_SPORTSMAN_1_ADDR			EEPROM_CALIB_MEMORY_START + 122
#define EEPROM_XBEE_SPORTSMAN_2_ADDR			EEPROM_CALIB_MEMORY_START + 130
#define EEPROM_XBEE_SPORTSMAN_3_ADDR			EEPROM_CALIB_MEMORY_START + 138
#define EEPROM_XBEE_SPORTSMAN_4_ADDR			EEPROM_CALIB_MEMORY_START + 146
#define EEPROM_XBEE_SPORTSMAN_5_ADDR			EEPROM_CALIB_MEMORY_START + 154
#define EEPROM_XBEE_SPORTSMAN_6_ADDR			EEPROM_CALIB_MEMORY_START + 162
#define EEPROM_XBEE_SPORTSMAN_7_ADDR			EEPROM_CALIB_MEMORY_START + 170
#define EEPROM_XBEE_SPORTSMAN_8_ADDR			EEPROM_CALIB_MEMORY_START + 178
#define EEPROM_XBEE_SPORTSMAN_9_ADDR			EEPROM_CALIB_MEMORY_START + 186
#define EEPROM_XBEE_SPORTSMAN_10_ADDR			EEPROM_CALIB_MEMORY_START + 194

#define EEPROM_XBEE_BOUY_1_ADDR					EEPROM_CALIB_MEMORY_START + 202
#define EEPROM_XBEE_BOUY_2_ADDR					EEPROM_CALIB_MEMORY_START + 210
#define EEPROM_XBEE_BOUY_3_ADDR					EEPROM_CALIB_MEMORY_START + 218
#define EEPROM_XBEE_BOUY_4_ADDR					EEPROM_CALIB_MEMORY_START + 226
*/
#define BNO055_ADDRESS	0x28

int8_t eeprom_write(uint16_t byte_addr, const uint8_t *txbuf, uint8_t txbytes);
int8_t eeprom_read(uint16_t byte_addr, uint8_t *rxbuf, uint8_t rxbytes);
void eeprom_check_i2c_bus(void);
void eeprom_read_hw_version(void);
void eeprom_write_hw_version(void);
void bno055_read_id(void);
void start_eeprom_module(void);

//Very useful defines
#define SIZEOF(s,m) ((size_t) sizeof(((s *)0)->m))

//Defines read and write macroses with addressing by offset in typedef,
//	without allocating memory for struct
#define EEPROM_READ(var,buff)   eeprom_read(offsetof(eeprom_map_t,var), buff, SIZEOF(eeprom_map_t,var))
#define EEPROM_WRITE(var, buff)   eeprom_write(offsetof(eeprom_map_t,var), buff, SIZEOF(eeprom_map_t,var))
#endif
