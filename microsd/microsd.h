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

void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]);

void start_microsd_module(void);
uint8_t fs_create_new_log(void);
uint8_t fs_write_file_header(void);
uint8_t fs_write_line(void);
uint8_t fs_check_space(void);
uint8_t fs_close_log(void);
uint8_t fs_continue_log(void);

#endif /* SD_MODULES_MICROSD_MICROSD_H_ */
