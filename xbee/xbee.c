/*
 * xbee.c
 *
 *  Created on: Mar 23, 2019
 *      Author: a-h
 */
#include "xbee.h"

void xbee_read(BaseSequentialStream* chp, int argc, char* argv){
	//(void)argv;
	if (argc == 2){
		uint8_t len;
		char at[3];
		uint8_t at2[9];
		memcpy(at, argv, 3);
		at2[8] = '\0';
		//chprintf(chp, "Reading %s command \n\r", at);
		chThdSleepMilliseconds(10);
		len = xbee_create_at_read_message(at, at2);
	    chprintf((BaseSequentialStream*)&SD1, "SPI %s %d\n\r", at2, len);
		spiAcquireBus(&SPID1);              // Acquire ownership of the bus.
		palClearLine(LINE_RF_868_CS);
		chThdSleepMilliseconds(1);
		spiSend(&SPID1, 8, at2); // send request
		//spiExchange(&SPID1, 2, at, rxbuf); // Atomic transfer operations.
		spiReleaseBus(&SPID1); // Ownership release.
		palSetLine(LINE_RF_868_CS);
		//len = xbee_create_at_read_message(argv[2], &txbuffer[0]);
		//xbee_send(&SPID1, at2, len);

	}else{
		chprintf(chp, "Usage: xbee read <AT command>\n\r \n\r");
	}
}

void xbee_write(BaseSequentialStream* chp, int argc, char* argv[]){
	if (argc == 3){
		uint8_t len;
		uint8_t txbuffer[50];
		char *at = argv[1];
		uint32_t var = atoi(argv[2]);
		chprintf(chp, "Write %s %x command \n\r", at, var);
		len = xbee_create_at_read_message(argv[2], &txbuffer[0]);

		xbee_send(&SPID1, &txbuffer[0], len);
	}else{
		chprintf(chp, "Usage: xbee write <AT command> <value>\n\r \n\r");
	}
}

void xbee_attn(BaseSequentialStream* chp, int argc, char* argv[]){
	if (argc == 1){
	uint8_t stat = palReadLine(LINE_RF_868_SPI_ATTN);
	chprintf(chp, "ATIIN pin: %d \n\r", stat);
	}else{
		chprintf(chp, "Usage: xbee attn\n\r \n\r");
	}
}

uint8_t xbee_create_at_read_message(char *at, uint8_t *buffer){
	uint8_t i = 0;
	buffer[0] = 0x7E;	// Start delimiter
	buffer[1] = 0x00;	// Length MSB
	buffer[2] = 0x04;	// Length LSB
	buffer[3] = 0x08;	// Frame type - AT command
	buffer[4] = at[0];	// Frame ID - it will return back
	buffer[5] = at[0];
	buffer[6] = at[1];	// AT command - two symbols
	buffer[7] = xbee_calc_CRC(&buffer[3], 4);
	return 8;	// Return length of packet
}

uint8_t xbee_create_at_write_message(char *at, uint8_t *buffer, uint8_t *data, uint8_t num){
	uint8_t i = 0;
	buffer[0] = 0x7E;	// Start delimiter
	buffer[1] = 0x00;	// Length MSB
	buffer[2] = 0x04;	// Length LSB
	buffer[3] = 0x08;	// Frame type - AT command
	buffer[4] = at[0];	// Frame ID - it will return back
	buffer[5] = at[0];
	buffer[6] = at[1];	// AT command - two symbols
	for (i = 0; i < num; i++){
		buffer[7+i++] = data[i];
	}
	buffer[7+num] = xbee_calc_CRC(&buffer[3], 4 + num);
	return 8 + num;		// Return length of packet
}

void xbee_send(SPIDriver *SPID, uint8_t *txbuf, uint8_t len){
	palSetLine(LINE_RED_LED);
	spiAcquireBus(SPID);              	/* Acquire ownership of the bus.    */
	palClearLine(LINE_RF_868_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, len, txbuf);
	//spiExchange(SPID, 8, rxbuf, txbuf); 			/* Atomic transfer operations.      */
	spiReleaseBus(SPID); 				/* Ownership release.               */
	palSetLine(LINE_RF_868_CS);
	chThdSleepMilliseconds(1);
	palClearLine(LINE_RED_LED);
}

uint8_t xbee_check_attn(void){
	uint8_t i = 0;
	uint8_t rxbuff[50];
	uint8_t txbuf[50];
	txbuf[0] = 0xFF;
	if (!palReadLine(LINE_RF_868_SPI_ATTN)){
				//palSetLine(LINE_GREEN_LED); // LED ON.
				spiAcquireBus(&SPID1);              // Acquire ownership of the bus.
				palClearLine(LINE_RF_868_CS);
				chThdSleepMilliseconds(1);
				while(!palReadLine(LINE_RF_868_SPI_ATTN)){
					spiExchange(&SPID1, 1, txbuf, &rxbuff[i++]); // Atomic transfer operations.
					spiReleaseBus(&SPID1); // Ownership release.
					chThdSleepMilliseconds(1);
					//palClearLine(LINE_GREEN_LED); // LED OFF
					chprintf(&SD1, "%x", &rxbuff[i]);
				}
				palSetLine(LINE_RF_868_CS);
				chThdSleepMilliseconds(1);
				return 1;
			}else{
				i = 0;
				return 0;
			}
}
uint8_t xbee_calc_CRC(uint8_t *buffer, uint8_t num){
	uint8_t i;
	uint16_t sum = 0;
	for (i = 0; i < num; i++){
		sum += buffer[i];
	}
	sum &= 0xFF;
	sum = (uint8_t)(0xFF - sum);
	return (uint8_t)sum;
}
