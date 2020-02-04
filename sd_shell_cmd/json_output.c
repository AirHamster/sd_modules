/*
 * json_output.c
 *
 *  Created on: Jan 17, 2020
 *      Author: a-h
 */

#include "json_output.h"

static jfes_config_t jfes_config;
static json_msg_t json_output_message;
static jfes_value_t json_boats_messages[JSON_NUM_OF_BOATS];
static jfes_value_t json_bouys_messages[JSON_NUM_OF_BOUYS];
static json_remote_devs_t *remote_devs;




void json_init(uint8_t num_of_boats, uint8_t num_of_bouys){
	jfes_config.jfes_malloc = malloc;
	jfes_config.jfes_free = free;

	json_output_message.type = jfes_object;
	json_output_message.data = jfes_array;

	remote_devs = calloc(1, sizeof(json_remote_devs_t));
}
