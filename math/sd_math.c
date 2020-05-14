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
#include "eeprom.h"
#ifdef USE_BMX160_MODULE
#include "bmx160_i2c.h"
extern bmx160_t bmx160;
extern struct bmm150_dev bmm;
extern volatile float beta;
#endif
#include "calibration.h"
extern windsensor_t *wind;
extern bno055_t *bno055;
extern ubx_nav_pvt_t *pvt_box;
extern float lastSensorValues[SIZE_BUFFER_VALUES];
extern rudder_t *r_rudder;
CalibrationParmDef paramSD;
extern calib_parameters_t calibrations;
float windAngleTarget = 0.0;
float hullSpeedTarget = 0.0;
float velocityMadeGoodTarget = 0.0;

static THD_WORKING_AREA(math_thread_wa, 4096*4);
static THD_FUNCTION(math_thread, arg);

void start_math_module(void){
	chThdCreateStatic(math_thread_wa, sizeof(math_thread_wa), NORMALPRIO + 2,
				math_thread, NULL);
}

static THD_FUNCTION( math_thread, p) {
	(void) p;
	chRegSetThreadName("Math Thd");
	calib_init_params();
	//math_init_calibration_params(&paramSD);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		math_copy_sensor_values(lastSensorValues);
		calculateValues(&calibrations);
		dataFiltering(&calibrations);
		calculateTargets();
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}



void math_copy_sensor_values(float *lastSensorValues) {
	lastSensorValues[AWA] = (float) wind->direction;
//	chprintf(SHELL_IFACE, "AWA: %f\r\n", lastSensorValues[AWA]);
	lastSensorValues[AWS] = wind->speed;
//	chprintf(SHELL_IFACE, "AWS: %f\r\n", lastSensorValues[AWS]);
	lastSensorValues[CMG] = (float) pvt_box->headMot;	//cog	gps
//	chprintf(SHELL_IFACE, "CMG: %f\r\n", lastSensorValues[CMG]);
	lastSensorValues[HDM] = bmx160.yaw;
//	chprintf(SHELL_IFACE, "HDM: %f\r\n", lastSensorValues[HDM]);
	lastSensorValues[HEEL] = bno055->d_euler_hpr.r;
//	chprintf(SHELL_IFACE, "HEEL: %f\r\n", lastSensorValues[HEEL]);
	lastSensorValues[PITCH] = bno055->d_euler_hpr.p;
//	chprintf(SHELL_IFACE, "PITCH: %f\r\n", lastSensorValues[PITCH]);
	lastSensorValues[SOG] = (float) (pvt_box->gSpeed * 0.0036);
//	chprintf(SHELL_IFACE, "SOG: %f\r\n", lastSensorValues[SOG]);
}
