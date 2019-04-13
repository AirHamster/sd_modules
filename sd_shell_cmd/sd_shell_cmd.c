#include "sd_shell_cmds.h"
#include <hal.h>
#include "shellconf.h"
#include <shell.h>
#include <string.h>
#include <stdlib.h>

enum output_threads output;


#include "xbee.h"



static const ShellCommand commands[] = {
    {"start", cmd_start},
    {"c", cmd_c},
	{"ublox", cmd_ublox},
	{"xbee", cmd_xbee},
	{"gyro", cmd_gyro},
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


void cmd_start(BaseSequentialStream* chp, int argc, char* argv[]){
	if (argc != 0){
	if (strcmp(argv[0], "test") == 0){
		//toggle_test_output();
		chprintf(chp, "Test started\n\r");
		return;
	}
	if (strcmp(argv[0], "gps") == 0){
		//toggle_gps_output();
		chprintf(chp, "GPS started\n\r");
		return;
	}
	if (strcmp(argv[0], "ypr") == 0){
		//toggle_ypr_output();
		return;
	}
	if (strcmp(argv[0], "gyro") == 0){
		//toggle_gyro_output();
		return;
	}
	}
	chprintf(chp, "Usage: start test|gps|ypr|gyro\n\r");
}

void cmd_c(BaseSequentialStream* chp, int argc, char* argv[]){
	//cancel_output();
	chprintf(chp, "Stopped all outputs\n\r");
}

void cmd_ublox(BaseSequentialStream* chp, int argc, char* argv[]){
	if (argc != 0){
	if (strcmp(argv[0], "lpf") == 0){
		if (argc == 2){
			if (strcmp(argv[1], "on")){
				//neo_toggle_lpf(true);
			}else if(strcmp(argv[1], "off")){
				//neo_toggle_lpf(false);
			}
		}else{
			//neo_get_lpf_status();
		}
		return;
	}
	if (strcmp(argv[0], "slas") == 0){
		if (argc == 2){
					if (strcmp(argv[1], "on")){
						//neo_toggle_slas(true);
					}else if(strcmp(argv[1], "off")){
						//neo_toggle_slas(false);
					}
				}else{
					//neo_get_slas_status();
				}
		return;
	}
	if (strcmp(argv[0], "sbas") == 0){
		if (argc == 2){
					if (strcmp(argv[1], "on")){
						//neo_toggle_sbas(true);
					}else if(strcmp(argv[1], "off")){
						//neo_toggle_sbas(false);
					}
				}else{
					//neo_get_sbas_status();
				}
		return;
	}
	if (strcmp(argv[0], "rtk") == 0){
		if (argc == 2){
					if (strcmp(argv[1], "on")){
						//neo_toggle_rtk(true);
					}else if(strcmp(argv[1], "off")){
					//	neo_toggle_rtk(false);
					}
				}else{
					//neo_get_rtk_status();
				}
		return;
	}
	if (strcmp(argv[0], "stat") == 0){
					//neo_get_status();
		return;
	}
	}
	chprintf(chp, "Usage: ublox lpf|slas|sbas|rtk|stat\n\r");
}

void cmd_gyro(BaseSequentialStream* chp, int argc, char* argv[]){
	if (argc != 0){
	if (strcmp(argv[0], "params") == 0){
		return;
	}
	}
	chprintf(chp, "Usage: start params|||\n\r");
}

void cmd_xbee(BaseSequentialStream* chp, int argc, char* argv[]) {
	if (argc != 0){
	if (strcmp(argv[0], "addr") == 0){
		xbee_get_addr();
			return;
	}
	if (strcmp(argv[0], "dest") == 0){
		if (argc == 2){
			//xbee_set_dest(atoi(argv[1]));
		}
			//xbee_get_dest();
		return;
	}
	if (strcmp(argv[0], "mesh") == 0){
		return;
	}
	if (strcmp(argv[0], "rssi") == 0){
		xbee_get_rssi();
		return;
	}
	if (strcmp(argv[0], "stat") == 0){
		xbee_get_stat();
		return;
	}
	if (strcmp(argv[0], "ping") == 0){
		//xbee_get_ping();
		return;
	}
	if (strcmp(argv[0], "lb") == 0){
		if (argc == 2){
			xbee_set_loopback(argv);
		}else{
			xbee_get_lb_status();
		}
			return;
		}
	}
	chprintf(chp, "Usage: xbee addr|dest|mesh|rssi|ping|stat|lb\n\r");
}

