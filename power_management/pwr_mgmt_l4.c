/*
 * pwr_mgmt_l4.c
 *
 *  Created on: Nov 27, 2019
 *      Author: a-h
 */


#include "config.h"
#include <hal.h>
#include "math.h"
#include <shell.h>
#include <string.h>
#include <stdlib.h>
#include "pwr_mgmt_l4.h"

static THD_WORKING_AREA(pwr_mgmt_thread_wa, 512);
static THD_FUNCTION( pwr_mgmt_thread, p) {
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while(true){

	}
	prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
}




void pwr_switch_dc_dc(dcdc_enum dcdc, dcdc_enum state){

}

void pwr_start_threads(void){

}

void pwr_stop_threads(void){

}
