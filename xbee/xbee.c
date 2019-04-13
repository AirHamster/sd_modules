/*
 * xbee.c
 *
 *  Created on: Mar 23, 2019
 *      Author: a-h
 */
#include "xbee.h"
#include <string.h>
extern thread_reference_t xbee_trp;
extern xbee_struct_t *xbee;
void xbee_read(SPIDriver *SPID, uint8_t rxlen, uint8_t *at_msg, uint8_t *rxbuff){
		uint8_t len, i;
		uint8_t txbuff[20];
		memset(txbuff, 0x00, 20);
		chprintf((BaseSequentialStream*)&SD1, "Reading %s command \n\r", at_msg);
		chThdSleepMilliseconds(10);
		len = xbee_create_at_read_message(at_msg, txbuff);
	    chprintf((BaseSequentialStream*)&SD1, "SPI ");
	    for (i = 0; i < len; i++){
	    	chprintf((BaseSequentialStream*)&SD1, "%x ", txbuff[i]);
	    }
	    chprintf((BaseSequentialStream*)&SD1, "\n\r");

	    xbee_send(SPID, len, txbuff);
		chThdSleepMilliseconds(10);
		xbee_receive(SPID, rxlen, rxbuff);
		spiReleaseBus(SPID); // Ownership release.
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

uint8_t xbee_create_at_read_message(uint8_t *at, uint8_t *buffer){
	uint8_t i = 0;
	buffer[0] = 0x7E;	// Start delimiter
	buffer[1] = 0x00;	// Length MSB
	buffer[2] = 0x04;	// Length LSB
	buffer[3] = XBEE_AT_FRAME;	// Frame type - AT command
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
	buffer[3] = XBEE_AT_FRAME;	// Frame type - AT command
	buffer[4] = at[0];	// Frame ID - it will return back
	buffer[5] = at[0];
	buffer[6] = at[1];	// AT command - two symbols
	for (i = 0; i < num; i++){
		buffer[7+i] = data[i];
	}
	buffer[7+num] = xbee_calc_CRC(&buffer[3], 4 + num);
	return 8 + num;		// Return length of packet
}

uint8_t xbee_create_data_read_message(uint8_t *at, uint8_t *buffer){
	uint8_t i = 0;
	buffer[0] = 0x7E;	// Start delimiter
	buffer[1] = 0x00;	// Length MSB
	buffer[2] = 0x04;	// Length LSB
	buffer[3] = XBEE_TRANSMIT_FRAME;	// Frame type - AT command
	buffer[4] = at[0];	// Frame ID - it will return back
	buffer[5] = at[0];
	buffer[6] = at[1];	// AT command - two symbols
	buffer[7] = xbee_calc_CRC(&buffer[3], 4);
	return 8;	// Return length of packet
}

uint8_t xbee_create_data_write_message(char *at, uint8_t *buffer, uint8_t *data, uint8_t num){
	uint8_t i = 0;
	buffer[0] = 0x7E;	// Start delimiter
	buffer[1] = (XBEE_HEADER_LEN + num + 2) << 8;	// Length MSB
	buffer[2] = (XBEE_HEADER_LEN + num + 2) & 0xFF;	// Length LSB
	buffer[3] = XBEE_TRANSMIT_FRAME;	// Frame type - AT command
	buffer[4] = at[0];	// Frame ID - it will return back
	buffer[5] = xbee->dest_addr_h;	// Destination address MSB
	buffer[6] = xbee->dest_addr_l;	// Destination address LSB
	buffer[7] = 0xFF;				// Reserved, 0xff required
	buffer[8] = 0xFE;				// Reserved, 0xfe required
	buffer[9] = 0;					// Broadcast radius (num of mesh hops)
	buffer[10] = 1 << 6;			// Delivery method (point-multipoint now)
	// Payload copying
	for (i = 0; i < num; i++){
		buffer[11+i] = data[i];
	}
	buffer[11+num] = xbee_calc_CRC(&buffer[3], 4 + num);
	return 8 + num;		// Return length of packet
}

void xbee_receive(SPIDriver *SPID, uint8_t len, uint8_t *rxbuf){
	uint8_t txbuf[len];
	memset(txbuf, 0xff, len);
	spiAcquireBus(SPID);              	/* Acquire ownership of the bus.    */
	palClearLine(LINE_RF_868_CS);
	chThdSleepMilliseconds(1);
	spiExchange(SPID, len, txbuf, rxbuf); // Atomic transfer operations.
	spiReleaseBus(SPID); 				/* Ownership release.               */
	palSetLine(LINE_RF_868_CS);
	chThdSleepMilliseconds(1);
}

void xbee_send(SPIDriver *SPID, uint8_t len, uint8_t *txbuf){
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


/*=============================================================
 *
 * Xbee thread wakeup functoins - callbacks for shell commands
 *
 ==============================================================*/
void xbee_get_addr(void){
	xbee->suspend_state = 0;
	chSysLockFromISR();
	chThdResumeI(&xbee_trp, (msg_t)XBEE_GET_OWN_ADDR);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();
}

void xbee_get_rssi(void){
	chSysLockFromISR();
	chThdResumeI(&xbee_trp, (msg_t)XBEE_GET_RSSI);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();
}

void xbee_get_stat(void){
	chSysLockFromISR();
	chThdResumeI(&xbee_trp, (msg_t)XBEE_GET_STAT);  /* Resuming the thread with message.*/
	chSysUnlockFromISR();
}

void xbee_get_lb_status(void){
	chprintf((BaseSequentialStream*)&SD1, "XBee loopback state: %d\r\n", xbee->loopback_mode);
}

void xbee_set_loopback(char* argv[]){
	if (strcmp(argv[1], "on") == 0){
		xbee->loopback_mode = true;
	}else if (strcmp(argv[1], "off") == 0){
		xbee->loopback_mode = false;
	}else{
		chprintf((BaseSequentialStream*)&SD1, "Usage: xbee lb <on|off>\n\r");
	}
}
void xbee_thread_execute(uint8_t command){
	xbee->suspend_state = 0;
	  chSysLock();
	  chThdResume(&xbee_trp, command);  /* Resuming the thread with message.*/
	  chSysUnlock();
}
/*========================================================
 *
 * Xbee communicating funcs - used by thread or standalone
 *
 =========================================================*/

void xbee_read_own_addr(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	uint32_t address_l, address_h;
	xbee_read(&SPID1, 8+6, "SL", packet);
	/*chprintf((BaseSequentialStream*)&SD1, "ADDR_L ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r");
	address_l = packet[8] << 24 | packet[9] << 16 | packet[10] << 8 | packet[11];*/
	xbee_read(&SPID1, 8+6, "SH", packet);
	/*	chprintf((BaseSequentialStream*)&SD1, "ADDR_H ");
			    for (i = 0; i < 15; i++){
			    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
			    }
			    chprintf((BaseSequentialStream*)&SD1, "\n\r");*/
	address_h = packet[8] << 24 | packet[9] << 16 | packet[10] << 8 | packet[11];
	xbee_str->own_addr_h = address_h;
	xbee_str->own_addr_l = address_l;
	chprintf((BaseSequentialStream*)&SD1, "ADDR is %x%x\n\r", address_h, address_l);
}

uint16_t xbee_read_last_rssi(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	xbee_read(&SPID1, 8+6, "DB", packet);
	chprintf((BaseSequentialStream*)&SD1, "RSSI ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r\n\r");
    return (packet[7] << 8) | packet[8];
}

uint16_t xbee_get_packet_payload(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	uint32_t address_l, address_h;
	xbee_read(&SPID1, 8+6, "NP", packet);
	chprintf((BaseSequentialStream*)&SD1, "Packet payload: ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r\n\r");
	xbee_str->packet_payload = ((packet[8] << 8) | packet[9]);
	return ((packet[8] << 8) | packet[9]);
}

uint16_t xbee_get_bytes_transmitted(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	uint32_t address_l, address_h;
	xbee_read(&SPID1, 8+6, "BC", packet);
	chprintf((BaseSequentialStream*)&SD1, "BC: ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r\n\r");
	xbee_str->bytes_transmitted = ((packet[8] << 8) | packet[9]);
	return ((packet[7] << 8) | packet[8]);
}

uint16_t xbee_get_good_packets_res(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	uint32_t address_l, address_h;
	xbee_read(&SPID1, 8+6, "GD", packet);
	chprintf((BaseSequentialStream*)&SD1, "GD: ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r\n\r");
	xbee_str->good_packs_res = ((packet[8] << 8) | packet[9]);
	return ((packet[8] << 8) | packet[9]);
}

uint16_t xbee_get_received_err_count(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	uint32_t address_l, address_h;
	xbee_read(&SPID1, 8+6, "ER", packet);
	chprintf((BaseSequentialStream*)&SD1, "ER: ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r\n\r");
	xbee_str->rec_err_count = ((packet[8] << 8) | packet[9]);
	return ((packet[8] << 8) | packet[9]);
}

uint16_t xbee_get_transceived_err_count(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	uint32_t address_l, address_h;
	xbee_read(&SPID1, 8+6, "TR", packet);
	chprintf((BaseSequentialStream*)&SD1, "TR: ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r\n\r");
	xbee_str->trans_errs = ((packet[8] << 8) | packet[9]);
	return ((packet[8] << 8) | packet[9]);
}

uint16_t xbee_get_unicast_trans_count(xbee_struct_t *xbee_str){
	uint8_t packet[15];
	uint8_t i;
	uint32_t address_l, address_h;
	xbee_read(&SPID1, 8+6, "UA", packet);
	chprintf((BaseSequentialStream*)&SD1, "UA: ");
		    for (i = 0; i < 15; i++){
		    	chprintf((BaseSequentialStream*)&SD1, "%x ", packet[i]);
		    }
		    chprintf((BaseSequentialStream*)&SD1, "\n\r\n\r");
	xbee_str->unicast_trans_count = ((packet[8] << 8) | packet[9]);
	return ((packet[8] << 8) | packet[9]);
}
