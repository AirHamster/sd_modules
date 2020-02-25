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
extern windsensor_t *wind;
extern bno055_t *bno055;
extern ubx_nav_pvt_t *pvt_box;
extern float lastSensorValues[SIZE_BUFFER_VALUES];
extern rudder_t *r_rudder;
CalibrationParmDef paramSD;
float windAngleTarget = 0.0;
float hullSpeedTarget = 0.0;
float velocityMadeGoodTarget = 0.0;

static THD_WORKING_AREA(math_thread_wa, 4096*4);
static THD_FUNCTION(math_thread, arg);

/**
 * @brief Start math threads
 */
void start_math_module(void){
	chThdCreateStatic(math_thread_wa, sizeof(math_thread_wa), NORMALPRIO + 2,
				math_thread, NULL);
}

/**
 * @brief Math thread
 * @param THD_FUNCTION( math_thread, p)
 */
static THD_FUNCTION( math_thread, p) {
	(void) p;
	chRegSetThreadName("Math Thd");
	math_init_calibration_params(&paramSD);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		math_copy_sensor_values(lastSensorValues);
		calculateValues(&paramSD);
		dataFiltering(&paramSD);
		calculateTargets();
		//calculateTargets(&windAngleTarget, &hullSpeedTarget,
		//&velocityMadeGoodTarget);
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

/**
 * @brief Read saved in EEPROM calibration parameters
 * @param param @p CalibrationParmDef struct
 */
void math_init_calibration_params(CalibrationParmDef *param) {
	EEPROM_READ(MATH_MEMORY.MATH_COMPASS_CORRECTION, (uint8_t*)&paramSD.CompassCorrection);
	//eeprom_read(EEPROM_MATH_COMPASS_CORRECTION, (uint8_t*)&paramSD.CompassCorrection, 4);
	//chThdSleepMilliseconds(10);
	//paramSD.CompassCorrection = 0.0;
	EEPROM_READ(MATH_MEMORY.MATH_HSP_CORRECTION, (uint8_t*)&paramSD.HSPCorrection);
	//eeprom_read(EEPROM_MATH_HSP_CORRECTION, (uint8_t*)&paramSD.HSPCorrection, 4);
	//chThdSleepMilliseconds(10);
	//paramSD.HSPCorrection = 0.0;
	EEPROM_READ(MATH_MEMORY.MATH_HEEL_CORRECTION, (uint8_t*)&paramSD.HeelCorrection);
	//eeprom_read(EEPROM_MATH_HEEL_CORRECTION, (uint8_t*)&paramSD.HeelCorrection, 4);
	//chThdSleepMilliseconds(10);
	//paramSD.HeelCorrection = -4.75;
	EEPROM_READ(MATH_MEMORY.MATH_DECLANATION_CORRECTION, (uint8_t*)&paramSD.MagneticDeclanation);
	//eeprom_read(EEPROM_MATH_DECLANATION_CORRECTION, (uint8_t*)&paramSD.MagneticDeclanation, 4);
	//chThdSleepMilliseconds(10);
	//paramSD.MagneticDeclanation = 0.0;
	EEPROM_READ(MATH_MEMORY.MATH_PITCH_CORRECTION, (uint8_t*)&paramSD.PitchCorrection);
	//eeprom_read(EEPROM_MATH_PITCH_CORRECTION, (uint8_t*)&paramSD.PitchCorrection, 4);
	//chThdSleepMilliseconds(10);
	//paramSD.PitchCorrection = -6.5625;
	EEPROM_READ(MATH_MEMORY.MATH_RUDDER_CORRECTION, (uint8_t*)&paramSD.RudderCorrection);
	//eeprom_read(EEPROM_MATH_RUDDER_CORRECTION, (uint8_t*)&paramSD.RudderCorrection, 4);
	//chThdSleepMilliseconds(10);
	//paramSD.RudderCorrection = 0.0;
	EEPROM_READ(MATH_MEMORY.MATH_WIND_CORRECTION, (uint8_t*)&paramSD.WindCorrection);
	//eeprom_read(EEPROM_MATH_WIND_CORRECTION, (uint8_t*)&paramSD.WindCorrection, 4);
	//chThdSleepMilliseconds(10);
	//paramSD.WindCorrection = -4.0;
	EEPROM_READ(MATH_MEMORY.MATH_WINSIZE1_CORRECTION, (uint8_t*)&paramSD.WindowSize1);
	//eeprom_read(EEPROM_MATH_WINSIZE1_CORRECTION, (uint8_t*)&paramSD.WindowSize1, 1);
	//chThdSleepMilliseconds(10);
	EEPROM_READ(MATH_MEMORY.MATH_WINSIZE2_CORRECTION, (uint8_t*)&paramSD.WindowSize2);
	//eeprom_read(EEPROM_MATH_WINSIZE2_CORRECTION, (uint8_t*)&paramSD.WindowSize2, 1);
	//chThdSleepMilliseconds(10);
	EEPROM_READ(MATH_MEMORY.MATH_WINSIZE3_CORRECTION, (uint8_t*)&paramSD.WindowSize3);
	//eeprom_read(EEPROM_MATH_WINSIZE3_CORRECTION, (uint8_t*)&paramSD.WindowSize3, 1);
/*
	eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_LEFT, (uint8_t*) &r_rudder->min_native,
			4);
	chThdSleepMilliseconds(10);
	eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_LEFT, (uint8_t*) &r_rudder->min_degrees, 4);
	chThdSleepMilliseconds(10);
	eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_CENTER,	(uint8_t*) &r_rudder->center_native, 4);
	chThdSleepMilliseconds(10);
	eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_CENTER, (uint8_t*) &r_rudder->center_degrees, 4);
	chThdSleepMilliseconds(10);
	eeprom_read(EEPROM_RUDDER_CALIB_NATIVE_RIGHT, (uint8_t*) &r_rudder->max_degrees,
			4);
	chThdSleepMilliseconds(10);
	eeprom_read(EEPROM_RUDDER_CALIB_DEGREES_RIGHT, (uint8_t*) &r_rudder->max_degrees, 4);

	chThdSleepMilliseconds(10);*/
}

/**
 * @brief Update sensors values in math data array
 * @param lastSensorValues Math data array pointer
 */
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
