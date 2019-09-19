/*
 * sd_math.c
 *
 *  Created on: Sep 19, 2019
 *      Author: a-h
 */


#include "nina-b3.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "sd_shell_cmds.h"
#include "lag.h"
#include "adc.h"
#include "windsensor.h"
#include "bno055.h"
#include "neo-m8.h"
#include "filters.h"
#include "sailDataMath.h"
#include "sd_math.h"

extern windsensor_t *wind;
extern bno055_t *bno055;
extern ubx_nav_pvt_t *pvt_box;
extern float lastSensorValues[SIZE_BUFFER_VALUES];
CalibrationParmDef paramSD;
float windAngleTarget = 0.0;
float hullSpeedTarget = 0.0;
float velocityMadeGoodTarget = 0.0;

static THD_WORKING_AREA(math_thread_wa, 4096*4);
static THD_FUNCTION(math_thread, arg);

void start_math_module(void){
	chThdCreateStatic(math_thread_wa, sizeof(math_thread_wa), NORMALPRIO,
				math_thread, NULL);
}

static THD_FUNCTION( math_thread, p) {
	(void) p;
	chRegSetThreadName("Math Thd");
	math_init_calibration_params(&paramSD);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		math_copy_sensor_values(lastSensorValues);
		calculateValues(&paramSD);
		 dataFiltering(&paramSD);
		 calculateTargets(&windAngleTarget, &hullSpeedTarget,
		 		  		&velocityMadeGoodTarget);
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

void math_init_calibration_params(CalibrationParmDef *param) {
	paramSD.CompassCorrection = 0.0;
	paramSD.HSPCorrection = 0.0;
	paramSD.HeelCorrection = 0.0;
	paramSD.MagneticDeclanation = 0.0;
	paramSD.PitchCorrection = 0.0;
	paramSD.RudderCorrection = 0.0;
	paramSD.WindCorrection = 0.0;
}

void math_copy_sensor_values(float *lastSensorValues) {
	lastSensorValues[AWA] = (float) wind->direction;
	lastSensorValues[AWS] = wind->speed;
	lastSensorValues[CMG] = (float) pvt_box->headMot;	//cog	gps
	lastSensorValues[HDM] = bno055->d_euler_hpr.h;
	lastSensorValues[HEEL] = bno055->d_euler_hpr.r;
	lastSensorValues[PITCH] = bno055->d_euler_hpr.p;
	lastSensorValues[SOG] = (float) (pvt_box->gSpeed * 0.0036);
}
