/*
 * windsensor.c
 *
 *  Created on: Aug 10, 2019
 *      Author: a-h
 */
#include "windsensor.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "shell.h"
#include "chprintf.h"

extern struct ch_semaphore usart1_semaph;
static windsensor_t wind_struct;
static uint16_t uart_temp;
windsensor_t *wind = &wind_struct;


/** @brief Driver default configuration.*/
static const SerialConfig wind_uart_cfg =
{
  19200,
  0,
  USART_CR2_STOP1_BITS,
  0
};

static const UARTConfig uart8_cfg =
{
	NULL,
	NULL,
	windstation_rx_buff_filled,
	windstation_char_recieved_async,
	windstation_rx_error,
	NULL,
	0,
	19200,
	0,
	USART_CR2_LINEN,
	0
};

thread_reference_t wind_trp = NULL;

static THD_WORKING_AREA(wind_thread_wa, 2048);
static THD_FUNCTION(wind_thread, p){
	(void)p;
	msg_t msg;
	char str[64];
	int i = 0;
	uint8_t scan_res = 0;
	uint16_t tmp;
	int32_t token;

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
		//uartStartReceive(&UARTD8, 1, &tmp);
	//	msg = uartReceiveTimeout(&UARTD8, 1, &tmp, TIME_S2I(1));
		chSysLock();
			msg = chThdSuspendS(&wind_trp);
		chSysUnlock();
		if (msg == UART_CHAR_RECEIVED){
			str[i] = uart_temp;
			if (str[i] == '\n'){

				scan_res = sscanf(str, "0R1,Dn=%dD,Dm=%dD,Dx=%dD,Sn=%d.%dM,Sm=%d.%dM,Sx=%d.%dM", &Dn, &Dm, &Dx, &Sn1, &Sn2, &Sm1, &Sm2, &Sx1, &Sx2);
		/*		chSemWait(&usart1_semaph);
				chprintf((BaseSequentialStream*)&SD1, "Char: i = %d scan_res = %d %s\n\r", i, scan_res, str);
				chSemSignal(&usart1_semaph);*/
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
		}else if (msg == UART_GENERIC_NOTIFY){
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "UART8 notify\n\r");
			chSemSignal(&usart1_semaph);
		}else if (msg == UART_ERROR){
			chSemWait(&usart1_semaph);
			chprintf((BaseSequentialStream*)&SD1, "UART8 error\n\r");
			chSemSignal(&usart1_semaph);
		}
/*
		token = sdGetTimeout(&SD8, TIME_S2I(2));

if (token == MSG_TIMEOUT){
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*)&SD1, "Timeout\n\r");
	chSemSignal(&usart1_semaph);

} else if (token == MSG_RESET){
	chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*)&SD1, "Queue reset\n\r");
	chSemSignal(&usart1_semaph);

}else{
	if (token != '\n'){
					str[i++] = (uint8_t)token;
				}else{
					str[i++] = (uint8_t)token;
					sscanf(str, "0R1,Dn=%dD,Dm=%dD,Dx=%dD,Sn=%d.%dM,Sm=%d.%dM,Sx=%d.%dM", &Dn, &Dm, &Dx, &Sn1, &Sn2, &Sm1, &Sm2, &Sx1, &Sx2);
					wind->direction = Dm;
					wind->speed = (float)(Sm1 + Sm2 / 10.0);

					memset(str, 0, 128);
					i = 0;
				}
} */
	}
}

void start_windsensor_module(void)
{

	//sdStart(&SD8, &wind_uart_cfg);

	chThdCreateStatic(wind_thread_wa, sizeof(wind_thread_wa), NORMALPRIO+2, wind_thread, NULL);
	uartStart(&UARTD8, &uart8_cfg);
}
static void windstation_char_recieved_async(UARTDriver *uartp, uint16_t c){
	(void)uartp;
	uart_temp = c;
	chSysLockFromISR();
	chThdResumeI(&wind_trp, (msg_t)UART_CHAR_RECEIVED);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();

}

static void windstation_rx_buff_filled(UARTDriver *uartp){
	(void)uartp;
	chSysLockFromISR();
	chThdResumeI(&wind_trp, (msg_t)UART_GENERIC_NOTIFY);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();
	return;
}

static void windstation_rx_error(UARTDriver *uartp, uartflags_t e){
	(void)uartp;
	chSysLockFromISR();
	chThdResumeI(&wind_trp, (msg_t)UART_ERROR);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();
	return;
}
