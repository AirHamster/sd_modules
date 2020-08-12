/*
 * bmx160_i2c.c
 *
 *  Created on: Jul 16, 2019
 *      Author: a-h
 */

/**
 * @file    bmx160_i2c.c
 * @brief   BMX160 Driver funcs.
 *
 * @addtogroup BMX160
 * @{
 */

#include "config.h"
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "math.h"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "bmi160.h"
#include "bmm150.h"
#include "bmx160_i2c.h"
#include "sd_shell_cmds.h"
#include "eeprom.h"
#include "MadgwickAHRS.h"
#include "bno055_i2c.h"
extern bno055_t *bno055;
#include "hmc6343_i2c.h"
extern hmc6343_t *hmc6343;
#include "BsxFusionLibrary.h"
#include "BsxLibraryCalibConstants.h"
#include "BsxLibraryConstants.h"
#include "BsxLibraryDataTypes.h"
#include "BsxLibraryErrorConstants.h"


#include "Fusion.h"

/* Fusion algo */

FusionBias fusionBias;
FusionAhrs fusionAhrs;

float samplePeriod = 0.01f; // replace this value with actual sample period in seconds

/**
 * @brief Sensitivity in degrees per second per lsb as specified in gyroscope datasheet
 */
FusionVector3 gyroscopeSensitivity = {
    .axis.x = 0.00762195f,
    .axis.y = 0.00762195f,
    .axis.z = 0.00762195f,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

/**
 * @brief Sensitivity in g per lsb as specified in accelerometer datasheet
 */
FusionVector3 accelerometerSensitivity = {
    .axis.x = 0.000061035f,
    .axis.y = 0.000061035f,
    .axis.z = 0.000061035f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

/**
 * @brief Hard-iron bias in uT
 */
FusionVector3 hardIronBias = {
    .axis.x = 0.75f,
    .axis.y = 2.96875f,
    .axis.z = -14.5f,
}; // replace these values with actual hard-iron bias in uT if known




BSX_U8 bsxLibConfAcc[] = { 37, 0, 3, 1, 0, 9, 12, 150, 0, 16, 60, 0, 1, 0, 1, 0,
		176, 4, 82, 3, 0, 0, 64, 65, 1, 1, 1, 1, 2, 2, 2, 3, 3, 1, 1, 180, 115 };
BSX_U8 bsxLibConfMag[] = { 39, 0, 2, 1, 20, 5, 20, 5, 196, 9, 6, 9, 112, 23, 0,
		0, 128, 61, 205, 204, 76, 63, 0, 0, 224, 64, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 134, 84 };
BSX_U8 bsxLibConfGyro[] = { 14, 0, 1, 1, 3, 9, 12, 136, 19, 16, 1, 1, 129, 46 };
BSX_U8 bsxLibConf[] = { 116, 6, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128,
		63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 51, 179,
		62, 205, 204, 12, 63, 205, 204, 12, 63, 51, 51, 51, 63, 51, 51, 51, 63,
		205, 204, 76, 63, 1, 0, 9, 4, 2, 23, 183, 209, 56, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 23, 183, 209, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 23, 183, 209, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 189, 55, 134, 53, 189, 55, 134, 53, 189, 55,
		134, 53, 0, 0, 0, 0, 0, 0, 16, 66, 232, 3, 5, 0, 45, 0, 132, 3, 176, 4,
		150, 0, 8, 150, 0, 13, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128,
		63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51, 51, 179,
		62, 205, 204, 12, 63, 205, 204, 12, 63, 51, 51, 51, 63, 51, 51, 51, 63,
		205, 204, 76, 62, 1, 6, 4, 1, 0, 5, 0, 65, 1, 64, 1, 36, 0, 120, 0, 4,
		1, 20, 20, 2, 2, 0, 4, 0, 0, 128, 63, 205, 204, 204, 61, 154, 153, 153,
		63, 205, 204, 204, 62, 205, 204, 204, 61, 1, 0, 20, 0, 16, 4, 120, 0, 8,
		0, 0, 5, 154, 153, 25, 63, 154, 153, 25, 63, 80, 0, 9, 0, 30, 0, 232, 3,
		80, 0, 65, 0, 4, 0, 4, 0, 0, 128, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 128, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 128, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 128, 64, 181, 254, 22, 55, 181, 254, 22, 55, 181, 254, 22, 55, 139,
		222, 169, 56, 0, 0, 224, 64, 13, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 205, 204, 204, 61,
		1, 9, 9, 3, 19, 50, 163, 4, 205, 12, 100, 40, 4, 13, 0, 1, 154, 153,
		153, 62, 154, 153, 153, 62, 205, 204, 204, 62, 154, 153, 25, 63, 154,
		153, 153, 62, 0, 0, 128, 62, 154, 153, 153, 62, 236, 81, 184, 62, 205,
		204, 76, 63, 205, 204, 76, 63, 205, 204, 76, 63, 205, 204, 76, 63, 205,
		204, 76, 62, 205, 204, 76, 62, 205, 204, 76, 62, 205, 204, 76, 62, 0,
		194, 184, 178, 62, 53, 250, 142, 60, 10, 0, 10, 0, 0, 2, 0, 10, 0, 80,
		119, 86, 61, 13, 0, 0, 128, 62, 143, 194, 245, 60, 10, 215, 163, 60,
		100, 128, 52, 45, 70, 1, 10, 0, 80, 0, 0, 0, 192, 63, 0, 0, 0, 64, 9, 2,
		0, 0, 200, 65, 0, 0, 128, 66, 0, 0, 128, 65, 0, 0, 192, 63, 205, 204,
		76, 61, 194, 184, 178, 61, 50, 37, 59, 24, 71, 0, 0, 160, 64, 154, 153,
		25, 63, 80, 119, 86, 61, 0, 1, 205, 204, 76, 63, 0, 0, 96, 64, 0, 0, 32,
		64, 205, 204, 204, 61, 4, 143, 194, 245, 60, 2, 1, 2, 3, 4, 1, 10, 176,
		4, 88, 2, 10, 215, 35, 60, 10, 0, 10, 0, 0, 0, 250, 67, 0, 0, 122, 68,
		0, 0, 160, 63, 0, 0, 72, 66, 0, 0, 128, 63, 0, 0, 128, 62, 205, 204,
		204, 61, 0, 0, 32, 66, 0, 0, 128, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 128, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 62, 0,
		36, 116, 73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 116, 73, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 116, 73, 0, 0, 192, 64, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192,
		64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 192, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 128, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 64, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 64, 0, 0,
		128, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 128, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 64, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 64, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 128, 64, 10, 215, 35, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 215, 35, 60, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 215, 35, 60, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 23,
		183, 209, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 23, 183, 209, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 23, 183, 209, 56, 0, 0, 128, 63, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 172, 197, 39, 55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 172, 197, 39, 55, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 172, 197,
		39, 55, 0, 36, 116, 73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 116, 73, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 116, 73, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 4, 3, 0, 0, 0, 0, 0, 0, 10, 3, 4, 25,
		64, 18, 24, 0, 64, 114, 8, 0, 13, 226, 109 };

initParam_t s_input;
ts_workingModes s_workingmodes;
ts_HWsensorSwitchList HWsensorSwitchList;
libraryinput_t libraryInput_ts;

volatile float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
extern volatile float q0;
extern volatile float q1;
extern volatile float q2;
extern volatile float q3;

extern struct ch_semaphore usart1_semaph;
extern struct ch_semaphore i2c1_semaph;
int8_t rslt = BMI160_OK;
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;
uint32_t sensortime_1 = 0;
ts_dataeulerf32 orientEuler_rad;

ts_dataxyzf32 rawGyroData;
ts_dataxyzf32 rawAccData;
ts_dataxyzf32 rawMagData;

/* Macros for frames to be read */

#define ACC_FRAMES	10 /* 10 Frames are available every 100ms @ 100Hz */
#define GYR_FRAMES	10
#define MAG_FRAMES	10
/* 10 frames containing a 1 byte header, 6 bytes of accelerometer,
 * 6 bytes of gyroscope and 8 bytes of magnetometer data. This results in
 * 21 bytes per frame. Additional 40 bytes in case sensor time readout is enabled */
#define FIFO_SIZE	250

/* Variable declarations */

struct bmi160_dev bmi;
struct bmm150_dev bmm;
bmx160_t bmx160;
//uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_aux_data aux_data[MAG_FRAMES];
struct bmm150_mag_data mag_data[MAG_FRAMES];
struct bmi160_sensor_data gyro_data[GYR_FRAMES], accel_data[ACC_FRAMES];

int8_t rslt;

/* Auxiliary function declarations */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);


static THD_WORKING_AREA(bmx160_thread_wa, 4096*2);
static THD_FUNCTION(bmx160_thread, arg);
static THD_WORKING_AREA(bmx160_calib_thread_wa, 4096*2);
static THD_FUNCTION(bmx160_calib_thread, arg);
const I2CConfig bmx160_i2c_cfg = {
  0x30420F13,
		//0x20E7112A,
 // 0x40B45B69,
  0,
  0
};

/**
 * @brief Start BMX160 threads
 */
void start_bmx160_module(void){

	chThdCreateStatic(bmx160_thread_wa, sizeof(bmx160_thread_wa), NORMALPRIO + 5, bmx160_thread, NULL);
	//chThdCreateStatic(bmx160_calib_thread_wa, sizeof(bmx160_calib_thread_wa), NORMALPRIO, bmx160_calib_thread, NULL);
}

/*
 * Thread to process data collection and filtering from MPU9250
 */

static THD_FUNCTION(bmx160_thread, arg) {

	(void) arg;
	uint8_t static_cal_update_cnt = 0;
	BSX_U8 usecasetick;
	BSX_U8 calibtick;
	BSX_U8 magAcc;
	float temp;
	float filter[60];
	float filterx[60];
	float filtery[60];
	float summ = 0.0;
	float summx = 0.0;
	float summy = 0.0;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;
	float true_mag_x;
	float true_mag_y;
	uint8_t filter_index = 0;
	uint8_t i;
	chRegSetThreadName("bmx160 Thread");
	chThdSleepMilliseconds(500);
	i2cStart(&GYRO_IF, &bmx160_i2c_cfg);
	bmx160_full_init();
	chThdSleepMilliseconds(2500);
	s_input.accelspec = (BSX_U8 *) &bsxLibConfAcc;
	s_input.magspec = (BSX_U8 *) &bsxLibConfMag;
	s_input.gyrospec = (BSX_U8 *) &bsxLibConfGyro;
	s_input.usecase = (BSX_U8 *) &bsxLibConf;

	if (bsx_init(&s_input) == 0) {
		//chprintf(SHELL_IFACE, "\r\nBSX library initialized\r\n");
		s_workingmodes.opMode = BSX_WORKINGMODE_NDOF_GEORV_FMC_OFF;
		bsx_set_workingmode(&s_workingmodes);
		//HWsensorSwitchList.acc
		bsx_get_hwdependency(s_workingmodes, &HWsensorSwitchList);
	} else {
		chprintf(SHELL_IFACE, "\r\nBSX library NOT initialized\r\n");
	}
	bmx160_read_calib_from_eeprom(&bmx160);
/*	hardIronBias.axis.x = bmx160.mag_offset.x;
	hardIronBias.axis.y = bmx160.mag_offset.y;
	hardIronBias.axis.z = bmx160.mag_offset.z;
	*/
	// Initialise gyroscope bias correction algorithm
	FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

	// Initialise AHRS algorithm
	FusionAhrsInitialise(&fusionAhrs, 1.5f); // gain = 0.5

	// Set optional magnetic field limits
	FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT

	systime_t prev = chVTGetSystemTime(); // Current system time.

	while (true) {
		/* To read both Accel and Gyro data */
		if (bmx160.calib_flag == 1) {
			bmx160_mag_calibration(bmx160.magBias, bmx160.magScale);
			bmx160.mag_offset.x = bmx160.magBias[0];
			bmx160.mag_offset.y = bmx160.magBias[1];
			bmx160.mag_offset.z = bmx160.magBias[2];
			bmx160_save_calib_to_eeprom(&bmx160);
			bmx160.calib_flag = 0;
		}
		hardIronBias.axis.x = bmx160.mag_offset.x;
		hardIronBias.axis.y = bmx160.mag_offset.y;
		hardIronBias.axis.z = bmx160.mag_offset.z;

		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel,
				&gyro, &bmi);
		rslt = bmm150_read_mag_data(&bmm);

		libraryInput_ts.acc.data.x = accel.x;
		libraryInput_ts.acc.data.y = accel.y;
		libraryInput_ts.acc.data.z = accel.z;
		libraryInput_ts.acc.time_stamp = accel.sensortime * 39; // <- timestamp resolution on BMX160 is 39us, hence multiplying by 39
		libraryInput_ts.acc.time_stamp = sensortime_1; // <- timestamp resolution on BMX160 is 39us, hence multiplying by 39

		libraryInput_ts.gyro.data.x = gyro.x;
		libraryInput_ts.gyro.data.y = gyro.y;
		libraryInput_ts.gyro.data.z = gyro.z;
		libraryInput_ts.gyro.time_stamp = sensortime_1;

		libraryInput_ts.mag.data.x = bmm.data.x;
		libraryInput_ts.mag.data.y = bmm.data.y;
		libraryInput_ts.mag.data.z = bmm.data.z;
		libraryInput_ts.mag.time_stamp = sensortime_1;
		sensortime_1 += 10 * 1000;
		bsx_dostep(&libraryInput_ts);
		bsx_get_magrawdata(&rawMagData);

		bmx160.gx = gyro.x;		//due to 250 deg/sec
		bmx160.gy = gyro.y;
		bmx160.gz = gyro.z;

		bmx160.ax = accel.x;	//due to 2g
		bmx160.ay = accel.y;
		bmx160.az = accel.z;

		bmx160.mx = rawMagData.x;	//microTesla to Gauss
		bmx160.my = rawMagData.y;
		bmx160.mz = rawMagData.z;

		// Calibrate gyroscope
		FusionVector3 uncalibratedGyroscope = {
				.axis.x = bmx160.gx, /* replace this value with actual gyroscope x axis measurement in lsb */
				.axis.y = bmx160.gy, /* replace this value with actual gyroscope y axis measurement in lsb */
				.axis.z = bmx160.gz, /* replace this value with actual gyroscope z axis measurement in lsb */
		};

		FusionVector3 calibratedGyroscope = FusionCalibrationInertial(
				uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY,
				gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

		// Calibrate accelerometer
		FusionVector3 uncalibratedAccelerometer = {
				.axis.x = bmx160.ax, /* replace this value with actual accelerometer x axis measurement in lsb */
				.axis.y = bmx160.ay, /* replace this value with actual accelerometer y axis measurement in lsb */
				.axis.z = bmx160.az, /* replace this value with actual accelerometer z axis measurement in lsb */
		};

		FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(
				uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY,
				accelerometerSensitivity, FUSION_VECTOR3_ZERO);

		// Calibrate magnetometer
		FusionVector3 uncalibratedMagnetometer = {
				.axis.x = bmx160.mx, /* replace this value with actual magnetometer x axis measurement in uT */
				.axis.y = bmx160.my, /* replace this value with actual magnetometer y axis measurement in uT */
				.axis.z = bmx160.mz, /* replace this value with actual magnetometer z axis measurement in uT */
		};

		FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(
				uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY,
				hardIronBias);

		// Update gyroscope bias correction algorithm
		calibratedGyroscope = FusionBiasUpdate(&fusionBias,
				calibratedGyroscope);

		// Update AHRS algorithm
		FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope,
				calibratedAccelerometer, calibratedMagnetometer, samplePeriod);

		// Print Euler angles
		FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(
				FusionAhrsGetQuaternion(&fusionAhrs));

		if (eulerAngles.angle.yaw < 0.0){
			eulerAngles.angle.yaw *= -1.0;
		}else{
			eulerAngles.angle.yaw = 360.0 - eulerAngles.angle.yaw;
		}
		eulerAngles.angle.yaw -= 90.0;
		if (eulerAngles.angle.yaw < 0.0){
			eulerAngles.angle.yaw += 360.0;
		}
		bmx160.yaw = eulerAngles.angle.yaw;
		bmx160.roll = eulerAngles.angle.pitch;
		bmx160.pitch = eulerAngles.angle.roll;
		//eulerAngles.angle.yaw = 180.0 - eulerAngles.angle.yaw;
	/*	chprintf(SHELL_IFACE, "Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n",
				eulerAngles.angle.roll, eulerAngles.angle.pitch,
				eulerAngles.angle.yaw);
*/
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(10));
	}
}

static THD_FUNCTION(bmx160_calib_thread, arg) {

	(void) arg;
	uint8_t static_cal_update_cnt = 0;
	BSX_U8 usecasetick;
	BSX_U8 calibtick;
	BSX_U8 magAcc;

	chRegSetThreadName("Calib Thread");
	chThdSleepMilliseconds(500);

	systime_t prev = chVTGetSystemTime(); // Current system time.


	while (true) {
			chThdSleepMilliseconds(2000);
		}

}

/**
 * @brief BMX160 chip and fusion algorithm initialization
 * @return
 */
int8_t bmx160_full_init(void) {
	 /* Initialize your host interface to the BMI160 */

	chThdSleepMilliseconds(2500);
	s_input.accelspec = (BSX_U8 *) &bsxLibConfAcc;
	s_input.magspec = (BSX_U8 *) &bsxLibConfMag;
	s_input.gyrospec = (BSX_U8 *) &bsxLibConfGyro;
	s_input.usecase = (BSX_U8 *) &bsxLibConf;

	if (bsx_init(&s_input) == 0) {
		chprintf(SHELL_IFACE, "\r\nBSX library initialized\r\n");
		s_workingmodes.opMode = BSX_WORKINGMODE_NDOF_GEORV_FMC_OFF;
		bsx_set_workingmode(&s_workingmodes);
		//HWsensorSwitchList.acc
		bsx_get_hwdependency(s_workingmodes, &HWsensorSwitchList);
	} else {
		chprintf(SHELL_IFACE, "\r\nBSX library NOT initialized\r\n");
	}
	bmx160_read_calib_from_eeprom(&bmx160);
/*	hardIronBias.axis.x = bmx160.mag_offset.x;
	hardIronBias.axis.y = bmx160.mag_offset.y;
	hardIronBias.axis.z = bmx160.mag_offset.z;
	*/
	// Initialise gyroscope bias correction algorithm
	FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

	// Initialise AHRS algorithm
	FusionAhrsInitialise(&fusionAhrs, 1.5f); // gain = 0.5

	// Set optional magnetic field limits
	FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT


	    /* This example uses I2C as the host interface */
	    bmi.id = BMI160_I2C_ADDR;
	    bmi.read = bmx160_I2C_bus_read;
	    bmi.write = bmx160_I2C_bus_write;
	    bmi.delay_ms = bmx160_delay_ms;
	    bmi.interface = BMI160_I2C_INTF;

	    /* The BMM150 API tunnels through the auxiliary interface of the BMI160 */
	    /* Check the pins of the BMM150 for the right I2C address */
	    bmm.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
	    bmm.intf = BMM150_I2C_INTF;
	    bmm.read = bmm150_aux_read;
	    bmm.write = bmm150_aux_write;
	    bmm.delay_ms = bmx160_delay_ms;
	    bmm.settings.data_rate = BMM150_DATA_RATE_30HZ;

	    rslt = bmi160_init(&bmi);
	    /* Check rslt for any error codes */

	    /* Configure the BMI160's auxiliary interface for the BMM150 */
	    bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
	    bmi.aux_cfg.aux_i2c_addr = bmm.dev_id;
	    bmi.aux_cfg.manual_enable = BMI160_ENABLE; /* Manual mode */
	    bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; /* 8 bytes */

	    rslt = bmi160_aux_init(&bmi);
	    /* Check rslt for any error codes */

	    rslt = bmm150_init(&bmm);
	    /* Check rslt for any error codes */

	    /* Configure the accelerometer */
	    bmi.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
	    bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	    bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	    bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	    /* Configure the gyroscope */
	    bmi.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
	    bmi.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
	    bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	    bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	    rslt = bmi160_set_sens_conf(&bmi);
	    /* Check rslt for any error codes */

	    /* Configure the magnetometer. The regular preset supports up to 100Hz in Forced mode */
	    bmm.settings.preset_mode = BMM150_PRESETMODE_REGULAR;
	    rslt = bmm150_set_presetmode(&bmm);
	    /* Check rslt for any error codes */

	    /* It is important that the last write to the BMM150 sets the forced mode.
	     * This is because the BMI160 writes the last value to the auxiliary sensor
	     * after every read */
	    bmm.settings.pwr_mode = BMM150_FORCED_MODE;
	    rslt = bmm150_set_op_mode(&bmm);
	    /* Check rslt for any error codes */

	    uint8_t bmm150_data_start = BMM150_DATA_X_LSB;
	    bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;
	    rslt = bmi160_set_aux_auto_mode(&bmm150_data_start, &bmi);
	    /* Check rslt for any error codes */

	    /* Link the FIFO memory location */
//	    fifo_frame.data = fifo_buff;
//	    fifo_frame.length = FIFO_SIZE;
//	    bmi.fifo = &fifo_frame;

	    /* Clear all existing FIFO configurations */
//	    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &bmi);
	    /* Check rslt for any error codes */

//	    uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_AUX |  BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
//	    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
	    /* Check rslt for any error codes */
}

/**
 *
 * @param bmx160
 * @return
 */
int8_t bmx160_save_calib_to_eeprom(bmx160_t *bmx160){
	EEPROM_WRITE(MAGN_MEMORY.MAGN_X_OFFSET, (uint8_t*)&bmx160->mag_offset.x);
	EEPROM_WRITE(MAGN_MEMORY.MAGN_Y_OFFSET, (uint8_t*)&bmx160->mag_offset.y);
	EEPROM_WRITE(MAGN_MEMORY.MAGN_Z_OFFSET, (uint8_t*)&bmx160->mag_offset.z);
	//eeprom_write(EEPROM_MAGN_X_OFFSET_ADDR, (uint8_t*)&bmx160->mag_offset.x, sizeof(float)*3);
}

/**
 *
 * @param bmx160
 * @return
 */
int8_t bmx160_read_calib_from_eeprom(bmx160_t *bmx160){
	EEPROM_READ(MAGN_MEMORY.MAGN_X_OFFSET, (uint8_t*)&bmx160->mag_offset.x);
	EEPROM_READ(MAGN_MEMORY.MAGN_Y_OFFSET, (uint8_t*)&bmx160->mag_offset.y);
	EEPROM_READ(MAGN_MEMORY.MAGN_Z_OFFSET, (uint8_t*)&bmx160->mag_offset.z);
	hardIronBias.axis.x = bmx160->mag_offset.x;
	hardIronBias.axis.y = bmx160->mag_offset.y;
	hardIronBias.axis.z = bmx160->mag_offset.z;
	//eeprom_read(EEPROM_MAGN_X_OFFSET_ADDR, (uint8_t*)&bmx160->mag_offset.x, sizeof(float)*3);
}

/**
 *
 * @param msec
 */
void bmx160_delay_ms(uint16_t msec){
	chThdSleepMilliseconds(msec);
}

/**
 *
 * @param dev_addr
 * @param reg_data
 * @param r_len
 * @return
 */
int8_t bmx160_read(uint8_t dev_addr, uint8_t *reg_data, uint8_t r_len) {
	msg_t status;
	uint8_t databuff[64];
	uint8_t register_addr = reg_data[0];
	memset(databuff, 0, 64);

	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterTransmitTimeout(&GYRO_IF, dev_addr, &register_addr, 1, databuff,
			r_len, TIME_INFINITE);
	i2cReleaseBus(&GYRO_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1, "Shit happened: status %d\r\n",
				i2cGetErrors(&GYRO_IF));
		chSemSignal(&usart1_semaph);
		i2c_restart(&GYRO_IF);
		return -1;
	}
	memcpy(reg_data, databuff, r_len);
	return 0;

}

/**
 *
 * @param dev_addr
 * @param reg_data
 * @param wr_len
 * @return
 */
int8_t bmx160_write(uint8_t dev_addr, uint8_t *reg_data, uint8_t wr_len) {
	uint8_t rxbuff[1];
	msg_t status;
	uint8_t databuff[64];
	//uint8_t register_addr = reg_data[0];
	memcpy(databuff, reg_data, wr_len);
	i2cAcquireBus(&GYRO_IF);
	status = i2cMasterTransmitTimeout(&GYRO_IF, dev_addr, databuff, wr_len,
			NULL, 0, TIME_INFINITE);
	i2cReleaseBus(&GYRO_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened: status is %d\r\n", i2cGetErrors(&GYRO_IF));
		chSemSignal(&usart1_semaph);
		i2c_restart(&GYRO_IF);
		return -1;
	}
	return 0;
}

/**
 *
 * @param dev_addr
 * @param reg_addr
 * @param reg_data
 * @param cnt
 * @return
 */
int8_t bmx160_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t bmx160_iERROR = bmx160_INIT_VALUE;
	uint8_t array[I2C_BUFFER_LEN];
	uint8_t stringpos = 0;

	array[0] = reg_addr;
	for (stringpos = 0; stringpos < cnt; stringpos++)
	{
		array[stringpos + 1] =
			*(reg_data + stringpos);
	}
	/*
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "bmx160_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* bmx160_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver bmx160_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	bmx160_write(dev_addr, array, cnt+1);
	return 0;
}

 /*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
int8_t bmx160_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	//int32_t bmx160_iERROR = bmx160_INIT_VALUE;
	uint8_t array[I2C_BUFFER_LEN] = {0};
	uint8_t stringpos = 0;

	array[0] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "bmx160_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * bmx160_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver bmx160_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	bmx160_read(dev_addr, array, cnt);
	for (stringpos = 0; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return 0;
}
/*
void i2c_restart(I2CDriver *i2cp)
{
	i2cStop(i2cp);
	chThdSleepMilliseconds(1);
	i2cStart (i2cp, &bmx160_i2c_cfg);
}
*/

/* Auxiliary function definitions */
/**
 *
 * @param id
 * @param reg_addr
 * @param reg_data
 * @param len
 * @return
 */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    (void) id; /* id is unused here */

    return bmi160_aux_read(reg_addr, reg_data, len, &bmi);
}

/**
 *
 * @param id
 * @param reg_addr
 * @param reg_data
 * @param len
 * @return
 */
int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    (void) id; /* id is unused here */

    return bmi160_aux_write(reg_addr, reg_data, len, &bmi);
}

/**
 *
 * @param dest1
 * @param dest2
 */
void bmx160_mag_calibration(float * dest1, float * dest2) {
	uint16_t ii = 0, sample_count = 0;
	float mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	float mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767,
			32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Mag Calibration: Wave device in all axis!\r\n");
	chSemSignal(&usart1_semaph);
	chThdSleepMilliseconds(2000);

	sample_count = 1200; // at 100 Hz ODR, new mag data is available every 10 ms
	for (ii = 0; ii < sample_count; ii++) {

		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &bmi);
		rslt = bmm150_read_mag_data(&bmm);
		/* check for the mag calibration status */
		//	bsx_get_magcalibaccuracy(&magAcc);

		libraryInput_ts.acc.data.x = accel.x;
		libraryInput_ts.acc.data.y = accel.y;
		libraryInput_ts.acc.data.z = accel.z;
		libraryInput_ts.acc.time_stamp = accel.sensortime*39; // <- timestamp resolution on BMX160 is 39us, hence multiplying by 39
		libraryInput_ts.acc.time_stamp = sensortime_1; // <- timestamp resolution on BMX160 is 39us, hence multiplying by 39

		libraryInput_ts.gyro.data.x = gyro.x;
		libraryInput_ts.gyro.data.y = gyro.y;
		libraryInput_ts.gyro.data.z = gyro.z;
		libraryInput_ts.gyro.time_stamp = sensortime_1;

		libraryInput_ts.mag.data.x = bmm.data.x;
		libraryInput_ts.mag.data.y = bmm.data.y;
		libraryInput_ts.mag.data.z = bmm.data.z;
		libraryInput_ts.mag.time_stamp = sensortime_1;
		sensortime_1 += 10 * 1000;
		bsx_dostep(&libraryInput_ts);
		bsx_get_magrawdata(&rawMagData);

		bmx160.mx = rawMagData.x;	//microTesla to Gauss
		bmx160.my = rawMagData.y;
		bmx160.mz = rawMagData.z;
/*
		chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "MX: %f, MY: %f, MZ: %f\r\n",
						bmx160.magCount[0], bmx160.magCount[1], bmx160.magCount[2]);
		chSemSignal(&usart1_semaph);
		 */
		if (ii % 120 == 0) {
			chSemWait(&usart1_semaph);
			chprintf(SHELL_IFACE, "Calibration progress: %d%%\r\n", (uint8_t)(((ii+1) / 120) * 10));
			chSemSignal(&usart1_semaph);
		}

		bmx160.magCount[0] = bmx160.mx;
		bmx160.magCount[1] = bmx160.my;
		bmx160.magCount[2] = bmx160.mz;

		for (int jj = 0; jj < 3; jj++) {
			if (bmx160.magCount[jj] > mag_max[jj])
				mag_max[jj] = bmx160.magCount[jj];
			if (bmx160.magCount[jj] < mag_min[jj])
				mag_min[jj] = bmx160.magCount[jj];
		}

		chThdSleepMilliseconds(11);  // at 100 Hz ODR, new mag data is available every 10 ms
	}

/*	chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1, "max x = %f, max y = %f, max z = %f\r\n", mag_max[0], mag_max[1], mag_max[2]);
		chprintf((BaseSequentialStream*) &SD1, "min x = %f, min y = %f, min z = %f\r\n", mag_min[0], mag_min[1], mag_min[2]);
	chSemSignal(&usart1_semaph);
	*/
// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

		dest1[0] =  mag_bias[0]; // save mag biases in G for main program
	dest1[1] =  mag_bias[1];
	dest1[2] =  mag_bias[2];

// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad / ( mag_scale[0]);
	dest2[1] = avg_rad / ( mag_scale[1]);
	dest2[2] = avg_rad / ( mag_scale[2]);

	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Offset X = %f, Offset Y = %f, Offset Z = %f\r\nScale X = %f, Scale Y = %f, Scale Z = %f\r\nMag Calibration done!\r\n",
				dest1[0], dest1[1], dest1[2], dest2[0], dest2[1], dest2[2]);
	chSemSignal(&usart1_semaph);
}
