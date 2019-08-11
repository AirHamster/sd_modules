/*
 * filesystem.c
 *
 *  Created on: Aug 10, 2019
 *      Author: a-h
 */

#include "ff.h"
#include "filesystem.h"


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

static FRESULT scan_files(BaseSequentialStream *chp, char *path) {
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
      }
      else {
        chprintf(chp, "%s/%s\r\n", path, fn);
      }
    }
  }
  return res;
}

void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]) {
  FRESULT err;
  uint32_t fre_clust;
  FATFS *fsp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: tree\r\n");
    return;
  }
  if (!fs_ready) {
    chprintf(chp, "File System not mounted\r\n");
    return;
  }
  err = f_getfree("/", &fre_clust, &fsp);
  if (err != FR_OK) {
    chprintf(chp, "FS: f_getfree() failed\r\n");
    return;
  }
  chprintf(chp,
           "FS: %lu free clusters with %lu sectors (%lu bytes) per cluster\r\n",
           fre_clust, (uint32_t)fsp->csize, (uint32_t)fsp->csize * 512);
  fbuff[0] = 0;
  scan_files(chp, (char *)fbuff);
}



/*
 * Card insertion event.
 */
void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]){
  FRESULT err;

  /*
   * On insertion SDC initialization and FS mount.
   */

  chprintf(chp,
    	             "FS: mmcConnect\r\n");
    chThdSleepMilliseconds(110);
#if HAL_USE_SDC
  if (sdcConnect(&SDCD1))
#else
  if (mmcConnect(&MMCD1))
#endif
    return;
  chprintf(chp,
  	             "FS: trying to mounting\r\n");
  chThdSleepMilliseconds(110);
  err = f_mount(&SDC_FS, "/", 1);
  if (err != FR_OK) {
	  chprintf(chp,
	             "FS: error mounting %d\r\n", err);
#if HAL_USE_SDC
    sdcDisconnect(&SDCD1);
#else
    mmcDisconnect(&MMCD1);
#endif
    return;
  }
  fs_ready = TRUE;
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/


uint8_t fs_create_new_log(void){

}

uint8_t fs_write_file_header(void){

}

uint8_t fs_write_line(void){

}

uint8_t fs_check_space(void){

}

uint8_t fs_close_log(void){

}

uint8_t fs_continue_log(void){

}
