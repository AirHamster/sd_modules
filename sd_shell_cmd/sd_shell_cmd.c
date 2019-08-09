#include "sd_shell_cmds.h"
#include <hal.h>
#include "shellconf.h"
#include <shell.h>
#include <string.h>
#include <stdlib.h>


extern void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]);
extern cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]);
output_struct_t output_struct;
output_struct_t *output = &output_struct;
#include "xbee.h"
#include "neo-m8.h"
#include "neo_ubx.h"

char *complete_buffer[16];
char history_buffer[128];
const int history_size = 128;


static const ShellCommand commands[] = {
		{"start", cmd_start},
		{"c", cmd_c},
		{"ublox", cmd_ublox},
		{"xbee", cmd_xbee},
		{"gyro", cmd_gyro},
		{"tree", cmd_tree},
		{"mount_sd", cmd_mount},
		{NULL, NULL}
};


static const ShellConfig shell_cfg1 = {
		(BaseSequentialStream*) &SHELL_SD,
		commands,
		history_buffer,
		32,
		complete_buffer
};



thread_t *cmd_init(void){
	return chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
			"shell", NORMALPRIO,
			shellThread, (void *)&shell_cfg1);
}




void cmd_start(BaseSequentialStream* chp, int argc, char* argv[]){
	if (argc != 0){
		if (strcmp(argv[0], "test") == 0){
			toggle_test_output();
			chprintf(chp, "Test started\n\r");
			return;
		}
		if (strcmp(argv[0], "gps") == 0){
			toggle_gps_output();
			chprintf(chp, "GPS started\n\r");
			return;
		}
		if (strcmp(argv[0], "ypr") == 0){
			toggle_ypr_output();
			return;
		}
		if (strcmp(argv[0], "gyro") == 0){
			toggle_gyro_output();
			return;
		}
	}
	chprintf(chp, "Usage: start test|gps|ypr|gyro\n\r");
}

void cmd_c(BaseSequentialStream* chp, int argc, char* argv[]){
	(void)argc;
	(void)argv;
	stop_all_tests();
	chprintf(chp, "Stopped all outputs\n\r");
}

void cmd_ublox(BaseSequentialStream* chp, int argc, char* argv[]){
	if (argc != 0){
		if (strcmp(argv[0], "cog_lpf") == 0){
			chprintf(chp, "Cog stat, argc %d\r\n", argc);
			if (argc == 2){
				if (strcmp(argv[1], "on") == 0){
					neo_toggle_cog_lpf(true);
				}else if(strcmp(argv[1], "off") == 0){
					neo_toggle_cog_lpf(false);
				}else if(strcmp(argv[1], "status") == 0){
					neo_get_lpf_status();
				}else if(strcmp(argv[1], "help") == 0){
					chprintf(chp, "Usage: ublox cog_lpf on/off | status | max_speed | max_pos_acc | lp_gain\r\n");
				}else{
					chprintf(chp, "Usage: ublox cog_lpf on|off\r\n");
				}
			}else if(argc == 3){
				if (strcmp(argv[1], "max_speed") == 0){
					neo_set_cog_speed(argv[2]);
				}else if (strcmp(argv[1], "max_pos_acc") == 0){
					neo_set_cog_pos_acc(argv[2]);
				}else if (strcmp(argv[1], "lp_gain") == 0){
					neo_set_cog_gain(argv[2]);
				}else{
					chprintf(chp, "Usage: ublox cog_lpf on/off | status | max_speed | max_pos_acc | lp_gain\r\n");
				}
			}else{
				chprintf(chp, "Usage: ublox cog_lpf on/off | status | max_speed | max_pos_acc | lp_gain\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "vel_lpf") == 0){
			if (argc == 2){
				if (strcmp(argv[1], "on") == 0){
					neo_toggle_vel_lpf(true);
				}else if(strcmp(argv[1], "off") == 0){
					neo_toggle_vel_lpf(false);
				}
			}else if (argc == 3){
				if (strcmp(argv[1], "lp_gain") == 0){
					neo_set_vel_gain(argv[2]);
				}else{
					chprintf(chp, "Usage: ublox vel_lpf lp_gain");
				}
			}else{
				neo_get_lpf_status();
			}
			return;
		}
		if (strcmp(argv[0], "slas") == 0){
			if (argc == 2){
				if (strcmp(argv[1], "on") == 0){
					neo_toggle_slas(true);
				}else if(strcmp(argv[1], "off") == 0){
					neo_toggle_slas(false);
				}else if(strcmp(argv[1], "status") == 0){
					neo_get_slas_sbas_status();
				}
			}else{
				chprintf(chp, "Usage: ublox slas on/off | status\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "sbas") == 0){
			if (argc == 2){
				if (strcmp(argv[1], "on") == 0){
					neo_toggle_sbas(true);
				}else if(strcmp(argv[1], "off") == 0){
					neo_toggle_sbas(false);
				}else if(strcmp(argv[1], "status") == 0){
					neo_get_slas_sbas_status();
				}
			}else{
				chprintf(chp, "Usage: ublox sbas on/off | status\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "rtk") == 0){
			if (argc == 2){
				if (strcmp(argv[1], "on") == 0){
					neo_toggle_rtk(true);
				}else if(strcmp(argv[1], "off") == 0){
						neo_toggle_rtk(false);
				}else if(strcmp(argv[1], "status") == 0){
					neo_get_rtk_status();
				}
			}else{
				chprintf(chp, "Usage: ublox sbas on/off | status\r\n");
			}
			return;
		}
		if (strcmp(argv[0], "stat") == 0){
			neo_get_full_status();
			return;
		}
		if (strcmp(argv[0], "rate") == 0){
					if (argc == 2){
						if (strcmp(argv[1], "status") == 0){
							neo_get_rate_status();
						}
					}else if (argc == 3){
						if (strcmp(argv[1], "set") == 0){
							neo_set_rate_ms(argv[2]);
						}
					}else{
						chprintf(chp, "Usage: ublox rate status | set < ms >\r\n");
					}
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
			xbee_get_ping();
			return;
		}
		if (strcmp(argv[0], "channels") == 0){
					xbee_get_channels();
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

void toggle_test_output(void){
	output->test = 1;
}

void toggle_gps_output(void){
	output->gps = (~output->gps) & 0x01;
}

void toggle_ypr_output(void){
	output->ypr = (~output->ypr) & 0x01;
}

void toggle_gyro_output(void){
	output->gyro = (~output->gyro) & 0x01;
}

void stop_all_tests(void){
	output->test = 0;
	output->gps = 0;
	output->ypr = 0;
	output->gyro = 0;
}


