/*
 * json_output.h
 *
 *  Created on: Jan 17, 2020
 *      Author: a-h
 */

#ifndef SD_MODULES_SD_SHELL_CMD_JSON_OUTPUT_H_
#define SD_MODULES_SD_SHELL_CMD_JSON_OUTPUT_H_

#define JSON_NUM_OF_BOATS		2
#define JSON_NUM_OF_BOUYS		1

typedef struct{
	float lat;
	float lon;
	float gSpeed;
	float pitch;
	float roll;
	float rudder;
	float rudder_deg;
	float log;
	float wind_speed;
	uint32_t distance;
	uint16_t yaw;
	uint16_t headMot;
	uint32_t wind_dir;
	uint8_t boat_num;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t sat;
	int8_t rssi;
	uint8_t bat;
}json_boat_data_t;

typedef struct{
	float lat;
	float lon;
	float gSpeed;
	float wave_height;
	uint32_t distance;
	uint16_t yaw;
	uint16_t headMot;
	uint32_t wind_dir;
	uint8_t bouy_num;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t sat;
	int8_t rssi;
	uint8_t bat;
}json_bouy_data_t;

typedef struct{
	json_boat_data_t boat[JSON_NUM_OF_BOATS];
	json_bouy_data_t bouy[JSON_NUM_OF_BOUYS];
}json_remote_devs_t;

typedef struct{
	jfes_value_t
};
#endif /* SD_MODULES_SD_SHELL_CMD_JSON_OUTPUT_H_ */
