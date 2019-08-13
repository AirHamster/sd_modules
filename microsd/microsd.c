/*
 * filesystem.c
 *
 *  Created on: Aug 10, 2019
 *      Author: a-h
 */
#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "shell.h"
#include "chprintf.h"
#include "microsd.h"
#include "config.h"
#include "ff.h"
#include "neo-m8.h"

extern ubx_nav_pvt_t *pvt_box;
static FRESULT scan_files(BaseSequentialStream *chp, char *path);
static void microsd_show_tree(BaseSequentialStream *chp);
static void microsd_show_free(BaseSequentialStream *chp);
static void write_test_file(BaseSequentialStream *chp);
static void verbose_error(BaseSequentialStream *chp, FRESULT err);
static uint8_t microsd_mount_fs(void);
static char* fresult_str(FRESULT stat);


static thread_reference_t microsd_trp = NULL;
/* Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).*/
static const SPIConfig hs_spicfg = { false, NULL, GPIOC, GPIOC_SD_CS, 0, 0 };

/* Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).*/
static const SPIConfig ls_spicfg = { false, NULL, GPIOC, GPIOC_SD_CS,
		SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0, 0 };

/* MMC/SD over SPI driver configuration.*/
static MMCConfig const portab_mmccfg = {&SPID3, &ls_spicfg, &hs_spicfg};
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

/*static FRESULT scan_files(BaseSequentialStream *chp, char *path) {
	static FILINFO fno;
	FRESULT res;
	DIR dir;
	size_t i;
	char *fn;

	res = f_opendir(&dir, path);
	if (res == FR_OK) {
		i = strlen(path);
		while (((res = f_readdir(&dir, &fno)) == FR_OK) && fno.fname[0]) {
			if (FF_FS_RPATH && fno.fname[0] == '.')
				continue;
			fn = fno.fname;
			if (fno.fattrib & AM_DIR) {
				*(path + i) = '/';
				strcpy(path + i + 1, fn);
				res = scan_files(chp, path);
				*(path + i) = '\0';
				if (res != FR_OK)
					break;
			} else {
				chprintf(chp, "%s/%s\r\n", path, fn);
			}
		}
	}
	return res;
}*/

FRESULT scan_files(BaseSequentialStream *chp, char *path) {
	FRESULT res;
	FILINFO fno;
	DIR dir;
	int fyear,fmonth,fday,fhour,fminute,fsecond;

	int i;
	char *fn;

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
			/*
			 * Read the Directory.
			 */
			res = f_readdir(&dir, &fno);
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
				chprintf(chp, "      %s/%s\r\n", path, fn);
			}
		}
	} else {
		chprintf(chp, "FS: f_opendir() failed\r\n");
	}
	return res;
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
	(void) argv;
	(void) argc;
	(void) chp;
//	chSysLock();
	chThdResume(&microsd_trp, (msg_t) MICROSD_SHOW_TREE); /* Resuming the thread with message.*/
//	chSysUnlock();
}

void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argv;
	(void) argc;
	(void) chp;
//	chSysLock();
	chThdResume(&microsd_trp, (msg_t) MICROSD_WRITE_FILE); /* Resuming the thread with message.*/
//	chSysUnlock();
}

void cmd_free(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void) argv;
	(void) argc;
	(void) chp;
	//	chSysLock();
	chThdResume(&microsd_trp, (msg_t) MICROSD_SHOW_FREE); /* Resuming the thread with message.*/
	//	chSysUnlock();
}
/*
 * Card insertion event.
 */
void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;
//	chSysLock();
	chThdResume(&microsd_trp, (msg_t)MICROSD_MOUNT_FS);  /* Resuming the thread with message.*/
//	chSysUnlock();
}

static THD_WORKING_AREA(microsd_thread_wa, 4096);
static THD_FUNCTION( microsd_thread, p) {
	(void) p;
	msg_t msg;
	chRegSetThreadName("MicroSD Thd");
	mmcStart(&MMCD1, &portab_mmccfg);
	while (true) {

		chSysLock();
		msg = chThdSuspendS(&microsd_trp);
		chSysUnlock();

		if (msg == MICROSD_WRITE_FILE) {
			write_test_file((BaseSequentialStream*) &SD1);
		} else if (msg == MICROSD_OPEN_FILE) {

		} else if (msg == MICROSD_CLOSE_FILE) {

		} else if (msg == MICROSD_SCAN_FILES) {

		} else if (msg == MICROSD_SHOW_TREE) {
			microsd_show_tree((BaseSequentialStream*) &SD1);
		} else if (msg == MICROSD_SHOW_FREE) {
			microsd_show_free((BaseSequentialStream*) &SD1);
		} else if (msg == MICROSD_MOUNT_FS) {

			microsd_mount_fs();
		}
	}
}

static void microsd_show_tree(BaseSequentialStream *chp){
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

void start_microsd_module(void) {
	mmcObjectInit(&MMCD1);
	chThdCreateStatic(microsd_thread_wa, sizeof(microsd_thread_wa), NORMALPRIO + 3, microsd_thread, NULL);
	chThdSleepMilliseconds(110);
}

static void write_test_file(BaseSequentialStream *chp) {
	FIL fsrc;   /* file object */
	FRESULT err;
	int written;

	/*
	 * Open the text file
	 */
	err = f_open(&fsrc, "hello.txt", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_open(\"hello.txt\") failed.\r\n");
		verbose_error(chp, err);
		return;
	} else {
		chprintf(chp, "FS: f_open(\"hello.txt\") succeeded\r\n");
	}
	/*
	 * Write text to the file.
	 */
	written = f_puts ("Hello World", &fsrc);
	if (written == -1) {
		chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") failed\r\n");
	} else {
		chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") succeeded\r\n");
	}
	/*
	 * Close the file
	 */
	f_close(&fsrc);
}

static void microsd_show_free(BaseSequentialStream *chp) {
	FRESULT err;
	uint32_t clusters;
	FATFS *fsp;
	//(void)argc;
	//(void)argv;

	err = f_getfree("/", &clusters, &fsp);
	if (err != FR_OK) {
		chprintf(chp, "FS: f_getfree() failed\r\n");
		return;
	}
	/*
	 * Print the number of free clusters and size free in B, KiB and MiB.
	 */
	chprintf(chp,"FS: %ju free clusters\r\n    %lu sectors per cluster\r\n",
		clusters, (uint32_t)SDC_FS.csize);
	chprintf(chp,"%llu B free\r\n",
		clusters * (uint64_t)SDC_FS.csize * (uint64_t)MMCSD_BLOCK_SIZE);
	chprintf(chp,"%llu KB free\r\n",
		(clusters * (uint64_t)SDC_FS.csize * (uint64_t)MMCSD_BLOCK_SIZE)/(1024));
	chprintf(chp,"%llu MB free\r\n",
		(clusters * (uint64_t)SDC_FS.csize * (uint64_t)MMCSD_BLOCK_SIZE)/(1024*1024));
}

DWORD get_fattime (void){
	uint32_t time = 0;
	uint8_t tmp;
	//If fullyResolved flag set in gps pvt data then we have true timestamp
	tmp = pvt_box->valid;
	if ( (pvt_box->valid & (1 << 2)) != 0)
	{
		time |= (uint8_t)(pvt_box->year - 1980) << 25;
		time |= (uint8_t)pvt_box->month << 21;
		time |= (uint8_t)pvt_box->day << 16;
		time |= (uint8_t)pvt_box->hour << 11;
		time |= (uint8_t)pvt_box->min << 5;
		time |= (uint8_t)pvt_box->sec >> 1;	//seconds should be divided by two
		chprintf((BaseSequentialStream*) &SD1, "FS: pvt_box->valid = %x\r\n", time);
		return time;
	}else{

		time |= 0 << 25;
		time |= 1 << 21;
		time |= 2 << 16;
		time |= 3 << 11;
		time |= 4 << 5;
		time |= 5;
		return time;
	}
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
