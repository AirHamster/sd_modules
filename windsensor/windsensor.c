/*
 * windsensor.c
 *
 *  Created on: Aug 10, 2019
 *      Author: a-h
 */
/**
 * @file    windsensor.c
 * @brief   Windsensor driver funcs.
 *
 * @addtogroup WIND
 * @{
 */
#include "config.h"
#include "windsensor.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

extern struct ch_semaphore usart1_semaph;
static uint16_t uart_temp;
windsensor_t *wind;


/** @brief USART driver default configuration.*/
static const SerialConfig wind_uart_cfg =
{
  19200,
  0,
  USART_CR2_STOP1_BITS,
  0
};

thread_reference_t wind_trp = NULL;

/**
 * @brief Windsensor parsing thread
 */
static THD_WORKING_AREA(wind_thread_wa, 2048);
static THD_FUNCTION(wind_thread, p){
	(void)p;
	msg_t msg;
	char str[64];
	int i = 0;
	uint8_t scan_res = 0;
	uint16_t tmp;
	int8_t token;

	int R=0;
	   int Dn=0;
	   int Dm=0;
	   int Dx=0;
	   int Sn1=0;
	   int Sm1=0;
	   int Sx1=0;
	   int Sn2=0;
	   int Sm2=0;
	   int Sx2=0;

	chRegSetThreadName("Wind Parse");
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*)&SD1, "Wind thd");
	chSemSignal(&usart1_semaph);
	memset(str, 0, 64);
	while (true) {
		token = sdGet(&SD8);
			str[i] = token;
			if (str[i] == '\n'){

				scan_res = sscanf(str, "0R1,Dn=%dD,Dm=%dD,Dx=%dD,Sn=%d.%dM,Sm=%d.%dM,Sx=%d.%dM", &Dn, &Dm, &Dx, &Sn1, &Sn2, &Sm1, &Sm2, &Sx1, &Sx2);
				if (scan_res == 9){
				wind->direction = (uint16_t)Dm;
				wind->speed = (float)(Sm1 + Sm2 / 10.0);
				}
				i = 0;
				memset(str, 0, 64);
			}else{
			i++;
			}
			if (i >= 64)
			{
				i = 0;
				memset(str, 0, 64);
			}
	}
}

/**
 * @brief Start windsensor thread
 */
void start_windsensor_module(void)
{

	sdStart(&SD8, &wind_uart_cfg);
	chThdCreateStatic(wind_thread_wa, sizeof(wind_thread_wa), NORMALPRIO, wind_thread, NULL);

}
