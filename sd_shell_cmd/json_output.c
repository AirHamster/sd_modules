/*
 * json_output.c
 *
 *  Created on: Jan 17, 2020
 *      Author: a-h
 */

#include "json_output.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "cJSON.h"

/*static jfes_config_t jfes_config;
static json_msg_t json_output_message;
*/

static cJSON *json_boat_message[JSON_NUM_OF_BOATS];
static cJSON *json_bouy_message[JSON_NUM_OF_BOUYS];
static json_remote_devs_t *remote_devs;

void json_init(uint8_t num_of_boats, uint8_t num_of_bouys) {
	uint8_t i = 0;
	char *string = NULL;
	cJSON *hour = NULL;
	cJSON *min = NULL;
	cJSON *sec = NULL;
	cJSON *lat = NULL;
	cJSON *lon = NULL;
	cJSON *speed = NULL;
	cJSON *dist = NULL;
	cJSON *yaw = NULL;
	cJSON *bno_yaw = NULL;
	cJSON *pitch = NULL;
	cJSON *roll = NULL;
	cJSON *headMot = NULL;
	cJSON *sat = NULL;
	cJSON *rudder = NULL;
	cJSON *rudder_deg = NULL;
	cJSON *log = NULL;
	cJSON *wind_dir = NULL;
	cJSON *wind_spd = NULL;
	cJSON *rssi = NULL;
	cJSON *bat = NULL;
	for (i = 0; i < num_of_boats; i++) {
		json_boat_message[i] = cJSON_CreateObject();
		//name = cJSON_CreateString(sprintf("boat_%d", i));
		//cJSON_AddItemToObject(json_boat_message[i], "name", name);
		hour = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "hour", hour);
		min = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "min", sec);
		sec = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "sec", sec);
		lat = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "lat", lon);
		lon = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "lat", lon);
		speed = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "speed", speed);
		dist = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "dist", dist);
		yaw = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "yaw", yaw);
		bno_yaw = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "bno_yaw", bno_yaw);
		pitch = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "pitch", pitch);
		roll = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "roll", roll);
		headMot = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "headMot", headMot);
		sat = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "sat", sat);
		rudder = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "rudder", rudder);
		rudder_deg = cJSON_CreateNumber(0);
		cJSON_AddItemToObject(json_boat_message[i], "rudder_deg", rudder_deg);
		string = cJSON_Print(json_boat_message[i]);
		chprintf(SHELL_IFACE, "%s\r\n", string);
	}

	for (i = 0; i < num_of_bouys; i++) {
		json_bouy_message[i] = cJSON_CreateObject();
	}

	remote_devs = calloc(1, sizeof(json_remote_devs_t));
	//remote_devs->boat = json_boat_message;
	//remote_devs->bouy = json_bouy_message;
}


