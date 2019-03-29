/*
 * neo-m8.c
 *
 *  Created on: Mar 19, 2019
 *      Author: a-h
 */

#include "neo-m8.h"

void neo_write_byte(SPIDriver *SPID, uint8_t reg_addr, uint8_t value) {
	uint8_t txbuf[2];
	txbuf[0] = reg_addr;
	txbuf[1] = value;
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, 2, txbuf); /* send request       */
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}

uint8_t neo_read_byte(SPIDriver *SPID, uint8_t reg_addr) {
	uint8_t value;
	reg_addr |= 0x80;	//0x80 indicates read operation
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiSend(SPID, 1, &reg_addr); /* send request       */
	spiReceive(SPID, 1, &value);
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
	return value;
}

void neo_read_bytes(SPIDriver *SPID, uint16_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, 100);
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	//neo_process_packet();
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}

/* Brief: function that finds sequence of 50 0xFF bytes
 *
 *	returns offset from first array byte to 0xFF sequence
 *	returns -1 if no 0xFF sequences found
 *	returns -2 if found not full part of 0xFF sequence
 */
int8_t neo_find_ff(uint8_t *buff, uint8_t num){
	uint8_t i;
	uint8_t ff_series = 0;
	for (i = 1; i < num; i++){
		if ((buff[i-1] == buff[i]) && (buff[i] == 0xFF)){
				ff_series++;
				if (ff_series == 49){
					return i - 49;	// Return offset
				}
		}else{
			ff_series = 0;
		}
	}
	if (i != 0)
	{
		return -2;
	}else{
		return -1;
	}
}
