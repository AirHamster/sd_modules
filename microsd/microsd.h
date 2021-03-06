/*
 * filesystem.h
 *
 *  Created on: Aug 10, 2019
 *      Author: a-h
 */

#ifndef SD_MODULES_MICROSD_MICROSD_H_
#define SD_MODULES_MICROSD_MICROSD_H_

#include "stdint.h"
#include "ch.h"
#include "hal.h"
#include "ff.h"

#define MICROSD_WRITE_FILE		1
#define MICROSD_OPEN_FILE		2
#define MICROSD_CLOSE_FILE		3
#define MICROSD_SCAN_FILES		4
#define MICROSD_MOUNT_FS		5
#define MICROSD_SHOW_TREE		6
#define MICROSD_SHOW_FREE		7
#define MICROSD_WRITE_SENSOR_LOG_LINE 8

enum microsd_commands{
	MICROSD_NONE = 0,
	MICROSD_MKFS,
	MICROSD_TREE,
	MICROSD_MOUNT,
	MICROSD_LS,
	MICROSD_CAT,
	MICROSD_WRITE_LOG,
	MICROSD_UPDATE_CALIBFILE,
	MICROSD_REMOVE,
	MICROSD_FREE,
	MICROSD_CLOSE_LOG
};

#define MICROSD_DEFAULT_STATE MICROSD_WRITE_LOG

typedef struct{
	uint8_t path_to_file[128];
	uint8_t file_created;
}microsd_t;

typedef struct{
	uint8_t change_req;
	uint8_t state_curr;
	uint8_t state_prev;
	uint8_t state_new;
}microsd_fsm_t;

void cmd_open(BaseSequentialStream *chp, int argc, char *argv[]);
//void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]);
void microsd_update_calibfile(void);

/**
 * Print file tree
 * @param chp Print output stream
 * @param argc Num of input arguments
 * @param argv Arguments vector
 */
void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]);
/**
 *
 * @param chp
 */
void microsd_show_tree(BaseSequentialStream *chp);

/**
 * Manual mount SD card
 * @param chp Print output stream
 * @param argc Num of input arguments
 * @param argv Arguments vector
 */
void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * Deleting file from SD card
 * @param chp Print output stream
 * @param argc Num of input arguments
 * @param argv Arguments vector
 */
void cmd_remove(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * Print available space
 * @param chp Print output stream
 * @param argc Num of input arguments
 * @param argv Arguments vector
 */
void cmd_free(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 *
 * @param chp Print output stream
 * @param argc Num of input arguments
 * @param argv Arguments vector
 */
void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * Format SD card
 * @param chp Print output stream
 * @param argc Num of input arguments
 * @param argv Arguments vector
 */
void cmd_mkfs(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * Same as tree
 * @param chp Print output stream
 * @param argc Num of input arguments
 * @param argv Arguments vector
 */
void cmd_cat(BaseSequentialStream *chp, int argc, char *argv[]);
//DWORD get_fattime (void);

/**
 * Start MicroSD threads
 */
void start_microsd_module(void);
/**
 *
 * @param new_state
 */
void fsm_from_ls(uint8_t new_state);
void fsm_switch_to_default_state(void);
void fsm_change_state(uint8_t state);
void fsm_new_state(uint8_t state);
void fsm_from_writing(uint8_t new_state);
void fsm_from_none(uint8_t new_state);
void fsm_from_mount(uint8_t new_state);
void fsm_from_free(uint8_t new_state);
void fsm_from_cat(uint8_t new_state);
void fsm_from_mkfs(uint8_t new_state);

/**
 * Creates new logfile
 * @return
 */
uint8_t fs_create_new_log(void);

/**
 * Writes csv header to file
 * @return
 */
uint8_t fs_write_file_header(void);

/**
 * Writes sensors data to file
 * @return
 */
uint8_t fs_write_line(void);

/**
 * Check and print available space on SD card
 * @return
 */
uint8_t fs_check_space(void);

/**
 * Close the logfile
 * @return
 */
uint8_t fs_close_log(void);

/**
 * Open logfile without writing header line
 * @return
 */
uint8_t fs_continue_log(void);

#endif /* SD_MODULES_MICROSD_MICROSD_H_ */
