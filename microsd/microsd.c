/*
 * filesystem.c
 *
 *  Created on: Aug 10, 2019
 *      Author: a-h
 */
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "microsd.h"
#include "ff.h"
#ifdef USE_UBLOX_GPS_MODULE
#include "neo-m8.h"
extern ubx_nav_pvt_t *pvt_box;
#endif
#ifdef USE_BNO055_MODULE
#include "bno055.h"
#include "bno055_i2c.h"
extern bno055_t *bno055;
#endif
#ifdef USE_WINDSENSOR_MODULE
#include "windsensor.h"
extern windsensor_t *wind;
#endif
#include "lag.h"
#include "adc.h"

#ifdef USE_BMX160_MODULE
#include "bmx160_i2c.h"
extern bmx160_t bmx160;
extern struct bmm150_dev bmm;
extern volatile float beta;
#endif

microsd_t *microsd;
microsd_fsm_t *microsd_fsm;
extern lag_t *r_lag;
extern rudder_t *r_rudder;
extern struct ch_semaphore usart1_semaph;
#include "sailDataMath.h";
extern CalibrationParmDef paramSD;

static FRESULT scan_files(BaseSequentialStream *chp, char *path);
//static void microsd_show_tree(BaseSequentialStream *chp);
static int8_t microsd_open_calibfile(FIL *file);
static void microsd_write_logfile_header(BaseSequentialStream *chp);
static int8_t microsd_add_new_calibfile(FIL *file);

static void write_test_file(BaseSequentialStream *chp);
static void microsd_open_logfile(BaseSequentialStream *chp);
static void verbose_error(BaseSequentialStream *chp, FRESULT err);
static void cat_file(BaseSequentialStream *chp, uint8_t *path);
static void remove_file(BaseSequentialStream *chp, uint8_t *path);
static void get_free_space(BaseSequentialStream *chp);
static void fsm_leave_state(uint8_t state_current);
static void fsm_enter_state(uint8_t state_new);
static int8_t microsd_create_filename_from_date(uint8_t *name_str);
static int8_t microsd_create_filename(uint16_t iteration, uint8_t *name_str);
static uint8_t microsd_mount_fs(void);
static char* fresult_str(FRESULT stat);
static int8_t microsd_create_filename_from_date(uint8_t *name_str);
static void microsd_write_sensor_log_line(BaseSequentialStream *chp);
static void microsd_close_logfile();
static FIL logfile;   /* file object */
static FIL calibfile; //file with calibration data update info
static uint8_t path_to_calibfile[32];
static uint8_t path_to_file[32];

extern lag_t *r_lag;
extern rudder_t *r_rudder;

thread_reference_t microsd_trp = NULL;
/* Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).*/
static const SPIConfig hs_spicfg = { false, NULL, GPIOD, GPIOD_SD_CS, 0, 0 };

/* Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).*/
static const SPIConfig ls_spicfg = { false, NULL, GPIOD, GPIOD_SD_CS,
		SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0, 0 };

/* MMC/SD over SPI driver configuration.*/
static MMCConfig const mmccfg = {&MICROSD_IF, &ls_spicfg, &hs_spicfg};
MMCDriver MMCD1;
/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
static FATFS SDC_FS;

/* FS mounted and ready.*/
static bool fs_ready = FALSE;

/* Generic large buffer.*/
static uint8_t fbuff[1024];

static THD_WORKING_AREA(microsd_thread_wa, 4096*3);
static THD_FUNCTION( microsd_thread, p) {
	(void) p;
	msg_t msg;
	chRegSetThreadName("MicroSD Thd");
	mmcObjectInit(&MMCD1);
	mmcStart(&MMCD1, &mmccfg);   // Configures and activates the MMC peripheral.
	//microsd_mount_fs();
	//microsd_open_logfile((BaseSequentialStream*) &SD1);
	//microsd_write_logfile_header((BaseSequentialStream*) &SD1);
	//microsd_fsm->state_new = MICROSD_WRITE_LOG;
	microsd_fsm->state_prev = MICROSD_NONE;
	microsd_fsm->state_curr = MICROSD_NONE;
	microsd_fsm->state_new = MICROSD_NONE;
	fsm_new_state(MICROSD_MOUNT);
	wdgReset(&WDGD1);
	chThdSleepMilliseconds(110);
	systime_t prev = chVTGetSystemTime(); // Current system time.
	while (true) {
		wdgReset(&WDGD1);
		if (microsd_fsm->change_req == 1){
			fsm_change_state(microsd_fsm->state_new);
		}
		switch (microsd_fsm->state_curr){
		case MICROSD_NONE:
			break;
		case MICROSD_CAT:
			cat_file(SHELL_IFACE, microsd->path_to_file);
			fsm_switch_to_default_state();
			break;
		case MICROSD_LS:
			memset(fbuff,0,sizeof(fbuff));
			scan_files(SHELL_IFACE, fbuff);
			fsm_switch_to_default_state();
			break;
		case MICROSD_WRITE_LOG:
			microsd_open_logfile(SHELL_IFACE);
			microsd_write_sensor_log_line(SHELL_IFACE);
			break;
		case MICROSD_FREE:
			get_free_space(SHELL_IFACE);
			fsm_switch_to_default_state();
			break;
		case MICROSD_MOUNT:
			microsd_mount_fs();
			fsm_switch_to_default_state();
			break;
		case MICROSD_UPDATE_CALIBFILE:
			if (microsd_open_calibfile(&calibfile) == 0) {
				microsd_add_new_calibfile(&calibfile);
			}
			fsm_switch_to_default_state();
			break;
		case MICROSD_REMOVE:
			remove_file(SHELL_IFACE, microsd->path_to_file);
			fsm_switch_to_default_state();
			break;
		case MICROSD_CLOSE_LOG:
			microsd_close_logfile();
			fsm_change_state(MICROSD_WRITE_LOG);
			break;
		default:
			fsm_switch_to_default_state();
			break;
		}
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(100));
	}
}

/*
 * Scan Files in a path and print them to the character stream.
 */
FRESULT scan_files(BaseSequentialStream *chp, char *path) {
	FRESULT res;
	FILINFO fno;
	DIR dir;
	int fyear,fmonth,fday,fhour,fminute,fsecond;
	int i;
	char *fn;
	FSIZE_t fz;
	chprintf(chp, "\r\n*\n");
#if _USE_LFN
	fno.lfname = 0;
	fno.lfsize = 0;
#endif
	/*
	 * Open the Directory.
	 */
	res = f_opendir(&dir, path);
	if (res == FR_OK) {
		/*
		 * If the path opened successfully.
		 */
		i = strlen(path);
		while (true) {
			//chprintf(chp, "openeddir\r\n", path);
			//chThdSleepMilliseconds(100);
			/*
			 * Read the Directory.
			 */
			res = f_readdir(&dir, &fno);
			//chprintf(chp, "readdir\r\n", path);
			/*
			 * If the directory read failed or the
			 */
			if (res != FR_OK || fno.fname[0] == 0) {
				break;
			}
			/*
			 * If the directory or file begins with a '.' (hidden), continue
			 */
			if (fno.fname[0] == '.') {
				continue;
			}
			fn = fno.fname;
			fz = fno.fsize;
			/*
			 * Extract the date.
			 */
			fyear = ((0b1111111000000000&fno.fdate) >> 9)+1980;
			fmonth= (0b0000000111100000&fno.fdate) >> 5;
			fday  = (0b0000000000011111&fno.fdate);
			/*
			 * Extract the time.
			 */
			fhour   = (0b1111100000000000&fno.ftime) >> 11;
			fminute = (0b0000011111100000&fno.ftime) >> 5;
			fsecond = (0b0000000000011111&fno.ftime)*2;
			/*
			 * Print date and time of the file.
			 */
			chprintf(chp, "%4d-%02d-%02d %02d:%02d:%02d ", fyear, fmonth, fday, fhour, fminute, fsecond);
			/*
			 * If the 'file' is a directory.
			 */
			if (fno.fattrib & AM_DIR) {
				/*
				 * Add a slash to the end of the path
				 */
				path[i++] = '/';
				strcpy(&path[i], fn);
				/*
				 * Print that it is a directory and the path.
				 */
				chprintf(chp, "<DIR> %s/\r\n", path);
				/*
				 * Recursive call to scan the files.
				 */
				res = scan_files(chp, path);
				if (res != FR_OK) {
					break;
				}
				path[--i] = 0;
			} else {
				/*
				 * Otherwise print the path as a file.
				 */
				chprintf(chp, "\t%lu", fz);
				chprintf(chp, "\t\t%s/%s\r\n", path, fn);
			}
		}
	} else {
		chprintf(chp, "FS: f_opendir() failed\r\n");
	}
	chprintf(chp, "\r\n*\n");
	return res;
}

void microsd_show_tree(BaseSequentialStream *chp){
	FRESULT err;
		uint32_t fre_clust;
		FATFS *fsp;

		if (!fs_ready) {
			chprintf(chp, "File System not mounted\r\n");
			return;
		}
		err = f_getfree("/", &fre_clust, &fsp);
		if (err != FR_OK) {
			chprintf(chp, "FS: f_getfree() failed\r\n");
			return;
		}
		chprintf(chp, "FS: %lu free clusters with %lu sectors (%lu bytes) per cluster\r\n",
				fre_clust, (uint32_t) fsp->csize, (uint32_t) fsp->csize * 512);
		fbuff[0] = 0;
		scan_files(chp, (char *) fbuff);
}

void cmd_mkfs(BaseSequentialStream *chp, int argc, char *argv[]) {
	FRESULT err;
	int partition;
	if (argc!=1) {
		chprintf(chp, "Usage: mkfs [partition]\r\n");
		chprintf(chp, "       Formats partition [partition]\r\n");
		return;
	}
	partition=atoi(argv[0]);
	chprintf(chp, "FS: f_mkfs(%d,0,0) Started\r\n",partition);
//	err = f_mkfs(argv[0], FM_ANY, 0, NULL, 0);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_mkfs() failed\r\n");
		verbose_error(chp, err);
		return;
	}
	chprintf(chp, "FS: f_mkfs() Finished\r\n");
	return;
}

void microsd_card_insert(void){
	microsd_mount_fs();

}

static uint8_t microsd_mount_fs(void){

FRESULT err;

	/*
	 * On insertion SDC initialization and FS mount.
	 */

	chprintf((BaseSequentialStream*) &SD1, "FS: mmcConnect\r\n");
	chThdSleepMilliseconds(110);
#if HAL_USE_SDC
	if (sdcConnect(&SDCD1))
#else
	if (mmcConnect(&MMCD1))
#endif
		return;
	chprintf((BaseSequentialStream*) &SD1, "FS: trying to mounting\r\n");
	chThdSleepMilliseconds(110);
	err = f_mount(&SDC_FS, "/", 1);
	if (err != FR_OK) {
		chprintf((BaseSequentialStream*) &SD1, "FS: error mounting %d\r\n", err);
#if HAL_USE_SDC
		sdcDisconnect(&SDCD1);
#else
		mmcDisconnect(&MMCD1);
#endif
		return;
	}
	fs_ready = TRUE;
	chprintf((BaseSequentialStream*) &SD1, "FS: mounted successfully\r\n");
	}

void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;
	/*
	 * Set the file path buffer to 0
	 */
	fsm_new_state(MICROSD_LS);
	//microsd_fsm->state_new = MICROSD_LS;
	//microsd_show_tree(chp);
	//memset(fbuff,0,sizeof(fbuff));
	//scan_files(chp, fbuff);
}

/*
 * Print a text file to screen
 */
void cmd_cat(BaseSequentialStream *chp, int argc, char *argv[]) {
	/*
	 * Print usage
	 */
	if (argc != 1) {
		chprintf(chp, "Usage: cat filename\r\n");
		chprintf(chp, "       Echos filename (no spaces)\r\n");
		return;
	}
	memset(microsd->path_to_file, 0, sizeof(microsd->path_to_file));
	memcpy(microsd->path_to_file, argv[0], strlen(argv[0]));
//	microsd_fsm->state_new = MICROSD_CAT;
	fsm_new_state(MICROSD_CAT);
}

void cmd_remove(BaseSequentialStream *chp, int argc, char *argv[]){
	/*
		 * Remove usege
		 */
		if (argc != 1) {
			chprintf(chp, "Usage: remove filename\r\n");
			return;
		}
		memset(microsd->path_to_file, 0, sizeof(microsd->path_to_file));
		memcpy(microsd->path_to_file, argv[0], strlen(argv[0]));
		fsm_new_state(MICROSD_REMOVE);
}

static void remove_file(BaseSequentialStream *chp, uint8_t *path){
	FRESULT err;

	err = f_unlink(path);
	if (err == FR_OK){
		chprintf(chp, "FS: f_unlink(%s) succeed.\r\n", path);
		return;
	}
	if (err != FR_OK){
		if (err == FR_NO_FILE){
			chprintf(chp, "FS: f_unlink(%s) failed. File not found.\r\n", path);
		}else{
			chprintf(chp, "FS: f_unlink(%s) failed.\r\n", path);
		}
	}
	return;
}
/*
 * Print a text file to screen
 */
static void cat_file(BaseSequentialStream *chp, uint8_t *path) {
	FRESULT err;
	FIL fsrc;   /* file object */
	char Buffer[4095*2];
	UINT ByteToRead=sizeof(Buffer);
	UINT ByteRead;
	systime_t prev, prev2, systime;

	/*
	 * Attempt to open the file, error out if it fails.
	 */
	//chprintf(chp, "opening %s\r\n", path);
	//chThdSleepMilliseconds(110);

	err=f_open(&fsrc, path, FA_READ | FA_WRITE);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_open(%s) failed.\r\n", path);
		verbose_error(chp, err);
		return;
	}
	//chprintf(chp, "opened\r\n");
	//chThdSleepMilliseconds(110);
	/*
	 * Do while the number of bytes read is equal to the number of bytes to read
	 * (the buffer is filled)
	 */
	do {
	//	prev = chVTGetSystemTime(); // Current system time.
		/*
		 * Clear the buffer.
		 */
		if (microsd_fsm->change_req){
			break;
		}
		memset(Buffer,0,sizeof(Buffer));
		/*
		 * Read the file.
		 */
		//chprintf(chp, "FS: f_read\r\n");
		//chThdSleepMilliseconds(110);

		err=f_read(&fsrc,Buffer,ByteToRead,&ByteRead);
		if (err != FR_OK) {
			chprintf(chp, "FS: f_read() failed\r\n");
			verbose_error(chp, err);
			f_close(&fsrc);
			return;
		}
	//	systime = chVTGetSystemTime(); // Current system time.
	//	prev2 = systime;
		chSemWait(&usart1_semaph);
		chprintf(chp, "%s", Buffer);
		chSemSignal(&usart1_semaph);
	//	systime = chVTGetSystemTime(); // Current system time.
//		chprintf(chp, "\r\nRead  Time:\t%lu\tmicroseconds\r\n", TIME_I2US(prev2 - prev));
	//	chprintf(chp, "Print Time:\t%lu\tmicroseconds\r\n", TIME_I2US(systime - prev2));
	//	chprintf(chp, "Read  throughput:\t%f\tkB/sec\r\n", 1000000.0/TIME_I2US(prev2 - prev)*8);
	//	chprintf(chp, "Print  throughput:\t%f\tkB/sec\r\n", 1000000.0/TIME_I2US(systime - prev2)*8);

		//chThdSleepMilliseconds(1000);
	} while (ByteRead>=ByteToRead);
	chprintf(chp,"\r\n");
	/*
	 * Close the file.
	 */
	f_close(&fsrc);
	return;
}

void cmd_open(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argv;
	(void) argc;
	(void) chp;
	chThdResume(&microsd_trp, (msg_t) MICROSD_OPEN_FILE); /* Resuming the thread with message.*/
}

void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argv;
	(void) argc;
	(void) chp;
	chThdResume(&microsd_trp, (msg_t) MICROSD_WRITE_FILE); /* Resuming the thread with message.*/
}

static void get_free_space(BaseSequentialStream *chp) {
	FRESULT err;

	uint32_t clusters;
	uint64_t bytes;
	float kbytes, mbytes, gbytes;
	FATFS *fsp;

	err = f_getfree("/", &clusters, &fsp);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_getfree() failed\r\n");
		return;
	}
	/*
	bytes = clusters * (uint32_t) SDC_FS.csize * (uint32_t) MMCSD_BLOCK_SIZE;
	kbytes = ((float)bytes) / (1024.0);
	mbytes = ((float)kbytes) / (1024.0 * 1024.0);
	gbytes = ((float)mbytes) / (1024.0 * 1024.0 * 1024.0);
	*/
	/*
	 * Print the number of free clusters and size free in B, KiB and MiB.
	 */
	chprintf(chp, "FS: %lu free clusters\r\n    %lu sectors per cluster\r\n",
			clusters, (uint32_t) SDC_FS.csize);
	//chprintf(chp,"%lu B free\r\n",
	 //clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE);
	 chprintf(chp,"%lu KB free\r\n",
	 (clusters * (uint32_t)SDC_FS.csize / 1024 * (uint32_t)MMCSD_BLOCK_SIZE));
	 chprintf(chp,"%lu MB free\r\n",
	 (clusters * (uint32_t)SDC_FS.csize / (1024*1024) * (uint32_t)MMCSD_BLOCK_SIZE));
	/*chprintf(chp, "%lu B free\r\n", bytes);
	chprintf(chp, "%f KB free\r\n", kbytes);
	chprintf(chp, "%f MB free\r\n", mbytes);
	chprintf(chp, "%f GB free\r\n", gbytes);*/
}

void cmd_free(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;
	fsm_new_state(MICROSD_FREE);
	//microsd_fsm->state_new = MICROSD_FREE;

}

/*
 * Card insertion event.
 */
void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;
	microsd_fsm->state_new = MICROSD_MOUNT;
	//microsd_mount_fs();
	//chThdResume(&microsd_trp, (msg_t)MICROSD_MOUNT_FS);  /* Resuming the thread with message.*/
}

void microsd_update_calibfile(void){
	//fsm_new_state(MICROSD_UPDATE_CALIBFILE);
}

static void microsd_write_sensor_log_line(BaseSequentialStream *chp) {
	FRESULT res;
	FILINFO fno;
	int written = 0;
	uint8_t megastring[256];
	if (microsd->file_created != 0) {
	memset(megastring, 0, 256);
	sprintf((char*)megastring, "%d-%d,%d,%d,%d,%f,%f,%f,%d,%d,%f,%f,%d,%f,%d,%f,%f\r\n",
			pvt_box->month, pvt_box->day, pvt_box->hour, pvt_box->min, pvt_box->sec, pvt_box->lat / 10000000.0f, pvt_box->lon / 10000000.0f,
			(float) (pvt_box->gSpeed * 0.0036), (uint16_t) (pvt_box->headMot / 100000), (uint16_t)bmx160.yaw, bno055->d_euler_hpr.p,
			bno055->d_euler_hpr.r, wind->direction, wind->speed, pvt_box->numSV, r_rudder->degrees, r_lag->meters);

	f_lseek(&logfile, f_size(&logfile));
	written = f_puts((char*)megastring, &logfile);

	if (written == -1) {
		chprintf(chp, "\r\nWriting failed. No card inserted or corrupted FS\r\n");
	} else {
		palToggleLine(LINE_ORANGE_LED);
		//chprintf(chp, "FS: f_puts %s to %s succeeded\r\n", megastring, path_to_file);
	}

	f_sync(&logfile);
	}
}

static void microsd_write_logfile_header(BaseSequentialStream *chp) {
	FRESULT res;
	FILINFO fno;
	int written;
	f_lseek(&logfile, f_size(&logfile));
	//if (microsd->file_created == 0) {
	written =
			f_printf(&logfile, "DATE,HOUR,MIN,SEC,LAT,LON,SOG,COG_GPS,HDG,PITCH,ROLL,AWA,AWS,SAT,RDR,LOG,LOG_DIR,TRIM1,TRIM2,TRIM3,TRIM4,BMP,SAIL_TIME\r\n");

	if (written == -1) {
		chprintf(chp, "\r\nWriting failed. No card inserted or corrupted FS\r\n");
	} else {
		//chprintf(chp, "FS: f_puts(\"Hello World\",\"%s\") succeeded\r\n", path_to_file);
	}
	f_sync(&logfile);
	//}
}

static void microsd_write_calibration_header(BaseSequentialStream *chp) {
	FRESULT res;
	FILINFO fno;
	int written;
	f_lseek(&logfile, f_size(&logfile));
	//if (microsd->file_created == 0) {
	written = f_printf(&logfile, "#Calibration parameters\r\n");
	written = f_printf(&logfile, "CompassCorrection:%f\tWindCorrection:%f\r\n",paramSD.CompassCorrection, paramSD.WindCorrection);
	written = f_printf(&logfile, "MagneticDeclanation:%f\tHSPCorrection:%f\r\n",paramSD.MagneticDeclanation,paramSD.HSPCorrection);
	written = f_printf(&logfile, "HeelCorrection:%f\tPitchCorrection:%f\r\n",paramSD.HeelCorrection, paramSD.PitchCorrection);
	written = f_printf(&logfile, "WindowSize1:%d\tWindowSize2:%d\r\n", paramSD.WindowSize1, paramSD.WindowSize2);
	written = f_printf(&logfile, "WindowSize3:%d\tRudderLeftNative:%f\r\n", paramSD.WindowSize3, r_rudder->native);
	written = f_printf(&logfile, "RudderLeftDegrees:%f\tRudderCenterNative:%f\r\n", r_rudder->min_degrees, r_rudder->center_native);
	written = f_printf(&logfile, "RudderCenterDegrees:%f\tRudderRightNative:%d\r\n", r_rudder->center_degrees, r_rudder->max_native);
	written = f_printf(&logfile, "RudderRightDegrees:%f\r\n", r_rudder->max_degrees);

	if (written == -1) {
		chprintf(chp, "\r\nWriting failed. No card inserted or corrupted FS\r\n");
	} else {
		//chprintf(chp, "FS: f_puts(\"Hello World\",\"%s\") succeeded\r\n", path_to_file);
	}
	f_sync(&logfile);
	//}
}
void start_microsd_module(void) {
	chThdCreateStatic(microsd_thread_wa, sizeof(microsd_thread_wa), NORMALPRIO + 4, microsd_thread, NULL);
}

static void write_test_file(BaseSequentialStream *chp) {
	FIL fsrc;   /* file object */
	FRESULT err;
	int written;

	/*
	 * Open the text file
	 */

	/*
	 * Write text to the file.
	 */
	f_lseek(&logfile, f_size(&logfile));
	written = f_printf (&logfile, "Hello World");
	if (written == -1) {
		chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") failed\r\n");
	} else {
		chprintf(chp, "FS: f_puts(\"Hello World\",\"%s\") succeeded\r\n", path_to_file);
	}
	f_sync(&logfile);
	/*
	 * Close the file
	 */
	//f_close(&logfile);
	//memset(&logfile, 0, sizeof(FIL));
}

static int8_t microsd_open_calibfile(FIL *file){
	FRESULT err;
	int8_t written;
	memset(path_to_calibfile, 0, 32);
	memcpy(path_to_calibfile, "calibfile.csv", strlen("calibfile.csv"));

	err = f_open(&calibfile, path_to_calibfile,
					FA_READ | FA_WRITE | FA_CREATE_NEW);
	if (err == FR_EXIST){
		err = f_open(&calibfile, path_to_calibfile,
							FA_READ | FA_WRITE | FA_OPEN_APPEND);
	} else if (err == FR_OK){
		written = f_printf(&calibfile, "YEAR,MONTH,DAY,HOUR,MIN,SEC,COMPASS_CORRECTION,HSP_CORRECTION,DECLANATION_CORRECTION,HEEL_CORRECTION,PITCH_CORRECTION,RUDDER_CORRECTION,WIND_CORRECTION,WINSIZE1_CORRECTION,WINSIZE2_CORRECTION,WINSIZE3_CORRECTION,RDR_NATIVE_LEFT,RDR_NATIVE_CENTER,RDR_NATIVE_RIGHT,RDR_DEGREES_LEFT,RDR_DEGREES_CENTER,RDR_DEGREES_RIGHT,LAG_CALIB_NUMBER\r\n");
			if (written == -1) {
				chprintf(SHELL_IFACE, "\r\nWriting failed. No card inserted or corrupted FS\r\n");
			}else {
				f_sync(&calibfile);
			}
	return 0;
	}
	if (err != FR_OK){

		chprintf(SHELL_IFACE, "FS: f_open(\"%s\") failed.\r\n", path_to_calibfile);
		verbose_error(SHELL_IFACE, err);
		return -1;
	}

}

static int8_t microsd_add_new_calibfile(FIL *file) {
	int8_t written;

	f_lseek(&calibfile, f_size(&calibfile));
	written =
			f_printf(&calibfile,
					"%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f\r\n",
					pvt_box->year, pvt_box->month, pvt_box->day, pvt_box->hour,
					pvt_box->min, pvt_box->sec, paramSD.CompassCorrection,
					paramSD.HSPCorrection, paramSD.MagneticDeclanation,
					paramSD.HeelCorrection, paramSD.PitchCorrection,
					paramSD.RudderCorrection, paramSD.WindCorrection,
					paramSD.WindowSize1, paramSD.WindowSize2,
					paramSD.WindowSize3, r_rudder->min_native,
					r_rudder->center_native, r_rudder->max_native,
					r_rudder->min_degrees, r_rudder->center_degrees,
					r_rudder->max_degrees, r_lag->calib_num);

	if (written == -1) {
		chprintf(SHELL_IFACE,
				"\r\nWriting failed. No card inserted or corrupted FS\r\n");
		return -1;
	}else{
		f_sync(&calibfile);
		f_close(&calibfile);
	}
return 0;
}

static void microsd_close_logfile()
{
	microsd->file_created = 0;
	fsm_change_state(MICROSD_CLOSE_LOG);
	//f_sync(&calibfile);
	//f_close(&calibfile);
}

static void microsd_open_logfile(BaseSequentialStream *chp) {
	FRESULT err;
	uint16_t i;
	//microsd_create_filename_from_date(path_to_file);
	i = 1;

	if ((pvt_box->valid & (1 << 2)) == 0) {
		return;
}
	if (microsd->file_created == 0) {

		memset(path_to_file, 0, 32);
		microsd_create_filename(i, path_to_file);
		//err = f_open(&logfile, path_to_file, FA_READ | FA_WRITE | FA_OPEN_APPEND);
		err = f_open(&logfile, path_to_file,
				FA_READ | FA_WRITE | FA_CREATE_NEW);

		while (err == FR_EXIST) {
			i++;
			microsd_create_filename(i, path_to_file);
			err = f_open(&logfile, path_to_file,
					FA_READ | FA_WRITE | FA_CREATE_NEW);
			microsd_write_calibration_header(SHELL_IFACE);
			microsd_write_logfile_header(SHELL_IFACE);
		}
	} else {
		err = f_open(&logfile, path_to_file,
				FA_READ | FA_WRITE | FA_OPEN_APPEND);
	}
	if (err != FR_OK) {
		chprintf(chp, "FS: f_open(\"%s\") failed.\r\n", path_to_file);
		verbose_error(chp, err);
		return;
	} else {
		//chprintf(chp, "FS: f_open(\"%s\") succeeded\r\n", path_to_file);
		microsd->file_created = 1;
	}
}


DWORD get_fattime (void){
	uint32_t time = 0;
	uint8_t tmp;

#ifdef USE_UBLOX_GPS_MODULE
	//If fullyResolved flag set in gps pvt data then we have true timestamp
	tmp = pvt_box->valid;
	if ((pvt_box->valid & (1 << 2)) != 0) {
		time |= (uint8_t) (pvt_box->year - 1980) << 25;
		time |= (uint8_t) pvt_box->month << 21;
		time |= (uint8_t) pvt_box->day << 16;
		time |= (uint8_t) pvt_box->hour << 11;
		time |= (uint8_t) pvt_box->min << 5;
		time |= (uint8_t) pvt_box->sec >> 1;//seconds should be divided by two
	} else {
		time |= 0 << 25;
		time |= 1 << 21;
		time |= 2 << 16;
		time |= 3 << 11;
		time |= 4 << 5;
		time |= 5;
	}
#else
	// Else use hardcoded time
	time |= 0 << 25;
	time |= 1 << 21;
	time |= 2 << 16;
	time |= 3 << 11;
	time |= 4 << 5;
	time |= 5;
#endif
	return time;
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

uint8_t fs_create_new_log(void) {

}

uint8_t fs_write_file_header(void) {

}

uint8_t fs_write_line(void) {

}

uint8_t fs_check_space(void) {

}

uint8_t fs_close_log(void) {

}

uint8_t fs_continue_log(void) {

}

static void verbose_error(BaseSequentialStream *chp, FRESULT err) {
	chprintf(chp, "\t%s.\r\n",fresult_str(err));
}

static char* fresult_str(FRESULT stat) {
	char str[255];
	memset(str,0,sizeof(str));
	switch (stat) {
		case FR_OK:
			return "Succeeded";
		case FR_DISK_ERR:
			return "A hard error occurred in the low level disk I/O layer";
		case FR_INT_ERR:
			return "Assertion failed";
		case FR_NOT_READY:
			return "The physical drive cannot work";
		case FR_NO_FILE:
			return "Could not find the file";
		case FR_NO_PATH:
			return "Could not find the path";
		case FR_INVALID_NAME:
			return "The path name format is invalid";
		case FR_DENIED:
			return "Access denied due to prohibited access or directory full";
		case FR_EXIST:
			return "Access denied due to prohibited access";
		case FR_INVALID_OBJECT:
			return "The file/directory object is invalid";
		case FR_WRITE_PROTECTED:
			return "The physical drive is write protected";
		case FR_INVALID_DRIVE:
			return "The logical drive number is invalid";
		case FR_NOT_ENABLED:
			return "The volume has no work area";
		case FR_NO_FILESYSTEM:
			return "There is no valid FAT volume";
		case FR_MKFS_ABORTED:
			return "The f_mkfs() aborted due to any parameter error";
		case FR_TIMEOUT:
			return "Could not get a grant to access the volume within defined period";
		case FR_LOCKED:
			return "The operation is rejected according to the file sharing policy";
		case FR_NOT_ENOUGH_CORE:
			return "LFN working buffer could not be allocated";
		case FR_TOO_MANY_OPEN_FILES:
			return "Number of open files > _FS_SHARE";
		case FR_INVALID_PARAMETER:
			return "Given parameter is invalid";
		default:
			return "Unknown";
	}
	return "";
}

static int8_t microsd_create_filename(uint16_t iteration, uint8_t *name_str){
	uint8_t buffer[10];
	memset(buffer, 0, 10);

	itoa(iteration, (char*) buffer, 10);
	strcat(buffer, ".csv");
	memcpy(name_str, buffer, strlen(buffer));
	return 0;
}

static int8_t microsd_create_filename_from_date(uint8_t *name_str) {

#ifdef USE_UBLOX_GPS_MODULE
	if ((pvt_box->valid & (1 << 2)) != 0) {
		uint8_t buffer[10];
		memset(buffer, 0, 10);
		itoa(pvt_box->year, (char*) buffer, 10);
		strcat(name_str, buffer);
		strcat(name_str, "-");
		itoa(pvt_box->month, buffer, 10);
		strcat(name_str, buffer);
		strcat(name_str, "-");
		itoa(pvt_box->day, buffer, 10);
		strcat(name_str, buffer);
		strcat(name_str, "-");
		itoa(pvt_box->hour, buffer, 10);
		strcat(name_str, buffer);
		strcat(name_str, "_");
		itoa(pvt_box->min, buffer, 10);
		strcat(name_str, buffer);
		strcat(name_str, "_");
		itoa(pvt_box->sec, buffer, 10);
		strcat(name_str, buffer);
		strcat(name_str, ".csv");
		return 0;
	} else {
		strcat(name_str, "1980-01-01-01_01_01.csv");
		return -1;
	}
#else
	strcat(name_str, "1980-01-01-01_01_01.csv");
	return -1;
#endif
}

void fsm_new_state(uint8_t state){
	microsd_fsm->state_new = state;
	microsd_fsm->change_req = 1;
}


static void fsm_leave_state(uint8_t state_current){
	switch (state_current) {
		case MICROSD_NONE:
			break;
		case MICROSD_MOUNT:
			break;
		case MICROSD_WRITE_LOG:
			f_sync(&logfile);
			f_close(&logfile);
			break;
		case MICROSD_CAT:
			break;
		case MICROSD_MKFS:
			break;
		case MICROSD_FREE:
			break;
		case MICROSD_LS:
			break;
		case MICROSD_UPDATE_CALIBFILE:
			break;
		case MICROSD_REMOVE:
			break;
		default:
			break;
		}
}

static void fsm_enter_state(uint8_t state_new){
	switch (state_new) {
		case MICROSD_NONE:
			break;
		case MICROSD_MOUNT:
			break;
		case MICROSD_WRITE_LOG:
			microsd_open_logfile((BaseSequentialStream*) &SD1);
		//	microsd_write_calibration_header((BaseSequentialStream*) &SD1);
		//	microsd_write_logfile_header((BaseSequentialStream*) &SD1);
			break;
		case MICROSD_CAT:
			break;
		case MICROSD_MKFS:
			break;
		case MICROSD_FREE:
			break;
		case MICROSD_LS:
			break;
		case MICROSD_UPDATE_CALIBFILE:
			break;
		case MICROSD_REMOVE:
			break;
		default:
			break;
		}

}

void fsm_change_state(uint8_t state) {
	microsd_fsm->change_req = 0;
	microsd_fsm->state_prev = microsd_fsm->state_curr;
	microsd_fsm->state_curr = MICROSD_NONE;	//atomic operation below
	fsm_leave_state(microsd_fsm->state_prev);
	fsm_enter_state(microsd_fsm->state_new);
/*
	switch (microsd_fsm->state_curr) {
	case MICROSD_NONE:
		fsm_from_none(state);
		break;
	case MICROSD_MOUNT:
		fsm_from_mount(state);
		break;
	case MICROSD_WRITE_LOG:
		fsm_from_writing(state);
		break;
	case MICROSD_CAT:
		fsm_from_cat(state);
		break;
	case MICROSD_FREE:
		fsm_from_free(state);
		break;
	case MICROSD_LS:
		fsm_from_ls(state);
		break;
	case MICROSD_UPDATE_CALIBFILE:
		fsm_from_update_calibfile(state);
		break;
	case MICROSD_REMOVE:
		fsm_from_remove(state);
		break;
	default:
		break;
	}
	*/
	microsd_fsm->state_curr = state;
}

void fsm_switch_to_default_state(void){
	fsm_new_state(MICROSD_DEFAULT_STATE);
}


