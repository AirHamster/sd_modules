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

#include "xbee.h"
extern struct ch_semaphore usart1_semaph;

extern xbee_remote_dev_t remote_dev[NUM_OF_SPORTSMAN_DEVICES + NUM_OF_BOUY_DEVICES];

void json_print_remote_dev_data(xbee_remote_dev_t *r_data){

	xbee_sportsman_data_t *sdata = NULL;
	xbee_bouy_data_t *bdata = NULL;
	xbee_trainer_data_t *tdata = NULL;

	if (r_data->type == DEV_TYPE_SPORTSMAN){
			sdata = r_data->rf_data;
		}else if (r_data->type == DEV_TYPE_BOUY){
			bdata = r_data->rf_data;
		}else if (r_data->type == DEV_TYPE_TRAINER){
			tdata = r_data->rf_data;
		}
	//chprintf(SHELL_IFACE, "DEV %d\r\n", r_data->type);
	chSemWait(&usart1_semaph);
	if ((r_data->type == DEV_TYPE_SPORTSMAN) || (r_data->type == DEV_TYPE_TRAINER)){
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
			chprintf(SHELL_IFACE, "\"rssi\":%d,\r\n\t\t\t", remote_dev[1].rssi);
			//chprintf(SHELL_IFACE, "\"rssi\":%d,\r\n\t\t\t", r_data->rssi);
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
	}else if (r_data->type == DEV_TYPE_TRAINER){
		chprintf(SHELL_IFACE, "\"hour\":%d,\r\n\t\t\t", tdata->hour);
		chprintf(SHELL_IFACE, "\"min\":%d,\r\n\t\t\t", tdata->min);
		chprintf(SHELL_IFACE, "\"sec\":%d,\r\n\t\t\t", tdata->sec);
		chprintf(SHELL_IFACE, "\"lat\":%f,\r\n\t\t\t", tdata->lat);
		chprintf(SHELL_IFACE, "\"lon\":%f,\r\n\t\t\t", tdata->lon);
		chprintf(SHELL_IFACE, "\"speed\":%f,\r\n\t\t\t", tdata->speed);
		chprintf(SHELL_IFACE, "\"dist\":%d,\r\n\t\t\t", tdata->dist);
		chprintf(SHELL_IFACE, "\"yaw\":%d,\r\n\t\t\t", tdata->yaw);
		//chprintf(SHELL_IFACE, "\"bno_yaw  \":%d,\r\n\t\t\t", (uint16_t)bno055->d_euler_hpr.h);
		chprintf(SHELL_IFACE, "\"pitch\":%f,\r\n\t\t\t", tdata->pitch);
		chprintf(SHELL_IFACE, "\"roll\":%f,\r\n\t\t\t", tdata->roll);
		chprintf(SHELL_IFACE, "\"headMot\":%d,\r\n\t\t\t", tdata->headMot);
		chprintf(SHELL_IFACE, "\"sat\":%d,\r\n\t\t\t", tdata->sat);
		//chprintf(SHELL_IFACE, "\"rudder\":%f,\r\n\t\t\t", sdata->rdr);
		chprintf(SHELL_IFACE, "\"rudder_deg\":%f,\r\n\t\t\t", tdata->rdr);
		chprintf(SHELL_IFACE, "\"log\":%f,\r\n\t\t\t", tdata->log);
		chprintf(SHELL_IFACE, "\"wind_dir\":%d,\r\n\t\t\t", tdata->wind_direction);
		chprintf(SHELL_IFACE, "\"wind_spd\":%f,\r\n\t\t\t", tdata->wind_speed);
		chprintf(SHELL_IFACE, "\"rssi\":%d,\r\n\t\t\t", r_data->rssi);
		chprintf(SHELL_IFACE, "\"bat\":0\r\n\t\t\t");
		chprintf(SHELL_IFACE, "}\r\n\t}");
	}
			chSemSignal(&usart1_semaph);
}
