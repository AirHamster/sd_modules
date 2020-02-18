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
#include "parson.h"
#include "frozen.h"

#include "xbee.h"
/*static jfes_config_t jfes_config;
static json_msg_t json_output_message;
*/

/*
static cJSON *json_boat_message[JSON_NUM_OF_BOATS];
static cJSON *json_bouy_message[JSON_NUM_OF_BOUYS];
static json_remote_devs_t *remote_devs;
*/
extern struct ch_semaphore usart1_semaph;
static JSON_Value *json_boat_message[JSON_NUM_OF_BOATS];
static JSON_Value *json_bouy_message[JSON_NUM_OF_BOUYS];
static json_remote_devs_t *remote_devs;


/*
void json_remote_devs_init(uint8_t num_of_boats, uint8_t num_of_bouys) {
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
		//chprintf(SHELL_IFACE, "%s\r\n", string);
	}

	for (i = 0; i < num_of_bouys; i++) {
		json_bouy_message[i] = cJSON_CreateObject();
		hour = cJSON_CreateNumber(0);
				cJSON_AddItemToObject(json_bouy_message[i], "hour", hour);
				min = cJSON_CreateNumber(0);
				cJSON_AddItemToObject(json_bouy_message[i], "min", sec);
				sec = cJSON_CreateNumber(0);
				cJSON_AddItemToObject(json_bouy_message[i], "sec", sec);
				lat = cJSON_CreateNumber(0);
				cJSON_AddItemToObject(json_bouy_message[i], "lat", lon);
				lon = cJSON_CreateNumber(0);
				cJSON_AddItemToObject(json_bouy_message[i], "lat", lon);
	}

	//remote_devs = calloc(1, sizeof(json_remote_devs_t));
	//remote_devs->boat = json_boat_message;
	//remote_devs->bouy = json_bouy_message;
}

*/

void json_print_remote_dev_data(xbee_remote_dev_t *r_data){

	xbee_sportsman_data_t *sdata = NULL;
	xbee_bouy_data_t *bdata = NULL;

	if (r_data->type == DEV_TYPE_SPORTSMAN){
			sdata = r_data->rf_data;
		}else if (r_data->type == DEV_TYPE_BOUY){
			bdata = r_data->rf_data;
		}

	chSemWait(&usart1_semaph);
	if (r_data->type == DEV_TYPE_SPORTSMAN){
		chprintf(SHELL_IFACE, "\r\n{\"msg_type\":\"boats_data\",\r\n\t\t\"boat_%d\":{\r\n\t\t\t", r_data->number);
	}else if (r_data->type == DEV_TYPE_BOUY){
		chprintf(SHELL_IFACE, "\r\n{\"msg_type\":\"bouy_data\",\r\n\t\t\"bouy_%d\":{\r\n\t\t\t", r_data->number);
	}
	if (r_data->type == DEV_TYPE_SPORTSMAN){
			chprintf(SHELL_IFACE, "\"hour\":%d,\r\n\t\t\t", sdata->hour);
			chprintf(SHELL_IFACE, "\"min\":%d,\r\n\t\t\t", sdata->min);
			chprintf(SHELL_IFACE, "\"sec\":%d,\r\n\t\t\t", sdata->sec);
			chprintf(SHELL_IFACE, "\"lat\":%f,\r\n\t\t\t", sdata->lat);
			chprintf(SHELL_IFACE, "\"lon\":%f,\r\n\t\t\t", sdata->lon);
			chprintf(SHELL_IFACE, "\"speed\":%f,\r\n\t\t\t", sdata->speed);
			chprintf(SHELL_IFACE, "\"dist\":%d,\r\n\t\t\t", sdata->dist);
			chprintf(SHELL_IFACE, "\"yaw\":%d,\r\n\t\t\t", sdata->yaw);
			//chprintf(SHELL_IFACE, "\"bno_yaw  \":%d,\r\n\t\t\t", (uint16_t)bno055->d_euler_hpr.h);
			chprintf(SHELL_IFACE, "\"pitch\":%f,\r\n\t\t\t", sdata->pitch);
			chprintf(SHELL_IFACE, "\"roll\":%f,\r\n\t\t\t", sdata->roll);
			chprintf(SHELL_IFACE, "\"headMot\":%d,\r\n\t\t\t", sdata->headMot);
			chprintf(SHELL_IFACE, "\"sat\":%d,\r\n\t\t\t", sdata->sat);
			//chprintf(SHELL_IFACE, "\"rudder\":%f,\r\n\t\t\t", sdata->rdr);
			chprintf(SHELL_IFACE, "\"rudder_deg\":%f,\r\n\t\t\t", sdata->rdr);
			chprintf(SHELL_IFACE, "\"log\":%f,\r\n\t\t\t", sdata->log);
			//chprintf(SHELL_IFACE, "\"wind_dir\":%d,\r\n\t\t\t", wind->direction);
			//chprintf(SHELL_IFACE, "\"wind_spd\":%f,\r\n\t\t\t", wind->speed);
			chprintf(SHELL_IFACE, "\"rssi\":%d,\r\n\t\t\t", r_data->rssi);
			chprintf(SHELL_IFACE, "\"bat\":0\r\n\t\t\t");
			chprintf(SHELL_IFACE, "}\r\n\t}");
	}else if (r_data->type == DEV_TYPE_BOUY){
		chprintf(SHELL_IFACE, "\"hour\":%d,\r\n\t\t\t", bdata->hour);
		chprintf(SHELL_IFACE, "\"min\":%d,\r\n\t\t\t", bdata->min);
		chprintf(SHELL_IFACE, "\"sec\":%d,\r\n\t\t\t", bdata->sec);
		chprintf(SHELL_IFACE, "\"lat\":%f,\r\n\t\t\t", bdata->lat);
		chprintf(SHELL_IFACE, "\"lon\":%f,\r\n\t\t\t", bdata->lon);
		chprintf(SHELL_IFACE, "\"sat\":%d,\r\n\t\t\t", bdata->sat);
		chprintf(SHELL_IFACE, "\"rssi\":%d,\r\n\t\t\t", r_data->rssi);
		chprintf(SHELL_IFACE, "\"bat\":0\r\n\t\t\t");
		chprintf(SHELL_IFACE, "}\r\n\t}");
	}
			chSemSignal(&usart1_semaph);
}

void json_remote_devs_init(uint8_t num_of_boats, uint8_t num_of_bouys) {
	uint8_t i = 0;
	int16_t numb = 0;
	char *string = NULL;
	char *string2 = NULL;
	JSON_Object *hour = NULL;
	JSON_Object *min = NULL;
	JSON_Object *sec = NULL;
	JSON_Object *lat = NULL;
	JSON_Object *lon = NULL;
	JSON_Object *speed = NULL;
	JSON_Object *dist = NULL;
	JSON_Object *yaw = NULL;
	JSON_Object *bno_yaw = NULL;
	JSON_Object *pitch = NULL;
	JSON_Object *roll = NULL;
	JSON_Object *headMot = NULL;
	JSON_Object *sat = NULL;
	JSON_Object *rudder = NULL;
	JSON_Object *rudder_deg = NULL;
	JSON_Object *log = NULL;
	JSON_Object *wind_dir = NULL;
	JSON_Object *wind_spd = NULL;
	JSON_Object *rssi = NULL;
	JSON_Object *bat = NULL;
	string = malloc(256);
	string2 = malloc(256);
	for (i = 0; i < num_of_boats; i++) {
		struct json_out out = JSON_OUT_BUF(string, 256);
		struct json_out out2 = JSON_OUT_BUF(string2, 256);
		numb = json_printf(&out, "{%Q: %d, x: [%B, %B], y: %Q}", "foo", 123, 0, -1, "hi");
		json_prettify(string, 25, &out2);
		chprintf(SHELL_IFACE, "%s\r\n", string2);
	}

	for (i = 0; i < num_of_bouys; i++) {

	}

	//remote_devs = calloc(1, sizeof(json_remote_devs_t));
	//remote_devs->boat = json_boat_message;
	//remote_devs->bouy = json_bouy_message;
}
