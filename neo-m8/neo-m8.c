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

void neo_read_bytes(SPIDriver *SPID, uint8_t num, uint8_t *rxbuf) {
	uint8_t *txbuf[num];
	memset(txbuf, 0xFF, 100);
	palSetLine(LINE_GREEN_LED); /* LED ON.                          */
	spiAcquireBus(SPID);              /* Acquire ownership of the bus.    */
	palClearLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	spiExchange(SPID, num, txbuf, rxbuf); /* Atomic transfer operations.      */
	spiReleaseBus(SPID); /* Ownership release.               */
	palSetLine(LINE_NEO_CS);
	chThdSleepMilliseconds(1);
	palClearLine(LINE_GREEN_LED); /* LED OFF.*/
}
