#include "sd_shell_cmds.h"
#include <hal.h>
#include <shell.h>
#include <string.h>
#include <stdlib.h>

#include "xbee.h"



static const ShellCommand commands[] = {
    {"xbee", cmd_xbee},
    {"attn", cmd_attn},
    {NULL, NULL}
};


static const ShellConfig shell_cfg1 = {
    (BaseSequentialStream*) &SHELL_SD,
	commands
};



thread_t *cmd_init(void){
	return chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
		                 "shell", NORMALPRIO + 10,
		                  shellThread, (void *)&shell_cfg1);
}


void cmd_xbee(BaseSequentialStream* chp, int argc, char* argv[]) {
	char p[10];
	memcpy(p, argv[1], strlen(argv[1]));
	chprintf(chp, "Usage: xbee read|write|attn <AT command>\n\r");
}


void cmd_attn(BaseSequentialStream* chp, int argc, char* argv[]) {
	(void)argc, argv;
	uint8_t stat = palReadLine(LINE_RF_868_SPI_ATTN);
	chprintf(chp, "ATTN: %d \n\r", stat);
}

