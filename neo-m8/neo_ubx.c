/*
 * neo_ubx.c
 *
 *  Created on: Apr 16, 2019
 *      Author: a-h
 */
#include "neo_ubx.h"

void neo_toggle_slas(uint8_t stat){
	if (stat == true){
		//neo_read_struct();
	}else{

	}
}

void neo_toggle_sbas(uint8_t stat){
	if (stat == true){

	}else{

	}
}

void neo_toggle_rtk(uint8_t stat){
	if (stat == true){

		}else{

		}
}

void neo_toggle_cog_lpf(uint8_t stat){
	if (stat == true){

	}else{

	}
}

void neo_toggle_vel_lpf(uint8_t stat){
	if (stat == true){

	}else{

	}
}

void neo_set_cog_speed(char* val){
	uint16_t speed = atoi(val);
	if ((speed > 0) & (speed < 100)){

	}else{

	}
}

void neo_set_cog_gain(char* val){
	uint16_t gain = atoi(val);
		if (gain <= 255){

	}else{

	}
}

void neo_set_cog_pos_acc(char* val){
	uint16_t acc = atoi(val);
		if ((acc > 0) & (acc < 100)){

	}else{

	}
}

void neo_set_vel_gain(char* val){
	uint16_t gain = atoi(val);
		if (gain <= 255){

	}else{

	}
}

void neo_get_lpf_status(void){

}

void neo_get_slas_sbas_status(void){

}

void neo_get_rtk_status(void){

}

void neo_get_full_status(void){

}
