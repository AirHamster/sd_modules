/*
 * bq2560x.c
 *
 *  Created on: Sep 12, 2019
 *      Author: a-h
 */
#include "config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "bq2560x.h"
#ifdef USE_UBLOX_GPS_MODULE
#include "neo-m8.h"
extern ubx_nav_pvt_t *pvt_box;
#endif
#ifdef USE_BNO055_MODULE
#include "bno055.h"
#include "bno055_i2c.h"
extern bno055_t *bno055;
#endif
#ifdef USE_WINDSENSOR_MODULE
#include "windsensor.h"
extern windsensor_t *wind;
#endif

charger_regs_t *charger_regs;
charger_t *charger;
extern struct ch_semaphore usart1_semaph;

#ifdef SD_MODULE_TRAINER
const I2CConfig charger_if_cfg = {
  0xD0D43C4C,
  0,
  0
};
#endif

#ifdef SENSOR_BOX
const I2CConfig charger_if_cfg = {
  0x10E37AFF,
  0,
  0
};
#endif

#ifdef PWR_CPU
static const I2CConfig charger_if_cfg = {
  0x10E37AFF,
  0,
  0
};

const charger_cfg_t charger_cfg = {
	INPUT_CURRENT_2000,
	FAST_CHARGE_CURRENT_2040,
	SYSTEM_MINIMUM_VOLTAGE_3V,
	PRECHARGE_CURRENT_300,
	TERMINATION_CURRENT_180
};

#endif

static THD_WORKING_AREA(charger_thread_wa, 256);
static THD_FUNCTION( charger_thread, p) {
	(void) p;
	uint8_t i;
	chRegSetThreadName("Charger Thd");
	i2cStart(&CHARGER_IF, &charger_if_cfg);
	charger_init(&CHARGER_IF, &charger_cfg);
	systime_t prev = chVTGetSystemTime(); // Current system time.

	charger_read_all_regs(charger_regs);
	charger_parse_status(charger_regs, charger);

	while (true) {
	//	if (charger->)

		charger_read_all_regs(charger_regs);
		charger_parse_status(charger_regs, charger);
	/*		chprintf(SHELL_IFACE,
					"\r\nReaded from charger:\tR0\tR1\tR2\tR3\tR4\tR5\tR6\tR7\tR8\tR9\tRA\tRB\r\n");
			chprintf(SHELL_IFACE,
					"Readed from charger:\t%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x\r\n\r\n",
					charger_regs->reg00, charger_regs->reg01, charger_regs->reg02, charger_regs->reg03,
					charger_regs->reg04, charger_regs->reg05, charger_regs->reg06, charger_regs->reg07,
					charger_regs->reg08, charger_regs->reg09, charger_regs->reg0A, charger_regs->reg0B);
*/
	//	charger_print_info(charger);
		prev = chThdSleepUntilWindowed(prev, prev + TIME_MS2I(1000));
	}
}

int8_t charger_print_info(charger_t *charger) {

	chprintf(SHELL_IFACE, "Charging:\t");
	if (charger->charge_status == REG08_CHRG_STAT_IDLE) {
		chprintf(SHELL_IFACE, " not charging\r\n");
	} else if (charger->charge_status == REG08_CHRG_STAT_PRECHG) {
		chprintf(SHELL_IFACE, " pre-charge\r\n");
	} else if (charger->charge_status == REG08_CHRG_STAT_FASTCHG) {
		chprintf(SHELL_IFACE, " fast charging\r\n");
	} else if (charger->charge_status == REG08_CHRG_STAT_CHGDONE) {
		chprintf(SHELL_IFACE, " charge completed\r\n");
	}

	chprintf(SHELL_IFACE, "VBUS status:\t");
	if (charger->vbus_attached) {
		chprintf(SHELL_IFACE, " attached\r\n");
	} else {
		chprintf(SHELL_IFACE, " not attached\r\n");
	}

	chprintf(SHELL_IFACE, "POWER status:\t");
	if (charger->power_good) {
		chprintf(SHELL_IFACE, " power good\r\n");
	} else {
		chprintf(SHELL_IFACE, " power not good\r\n");
	}

}

int8_t charger_init(I2CDriver *i2cp, charger_cfg_t *cfg){
	int8_t status = 0;
	charger_regs_t regs;
	uint8_t temp;
	charger_read_all_regs(&regs);

	//Clear bits and write needed config
	temp = (regs.reg00 & ~REG00_IINLIM_MASK) | cfg -> input_current;
	charger_write_register(BQ2560X_REG_00, &temp, 1);

	temp = (regs.reg01 & ~REG01_SYS_MINV_MASK) | cfg -> system_min_voltage;
	charger_write_register(BQ2560X_REG_01, &temp, 1);

	temp = cfg->precharge_current | cfg->termination_current;
	charger_write_register(BQ2560X_REG_03, &temp, 1);

	//Not in enum yet
	/*
	temp = (regs.reg04 & ~REG04_VREG_MASK) | cfg -> charge_voltage;
	charger_write_register(BQ2560X_REG_04, &temp, 1);
*/


}

int8_t charger_read_all_regs(charger_regs_t *regs){
	int8_t status = 0;
	status |= charger_read_register(BQ2560X_REG_00, &charger_regs->reg00);
	status |= charger_read_register(BQ2560X_REG_01, &charger_regs->reg01);
	status |= charger_read_register(BQ2560X_REG_02, &charger_regs->reg02);
	status |= charger_read_register(BQ2560X_REG_03, &charger_regs->reg03);
	status |= charger_read_register(BQ2560X_REG_04, &charger_regs->reg04);
	status |= charger_read_register(BQ2560X_REG_05, &charger_regs->reg05);
	status |= charger_read_register(BQ2560X_REG_06, &charger_regs->reg06);
	status |= charger_read_register(BQ2560X_REG_07, &charger_regs->reg07);
	status |= charger_read_register(BQ2560X_REG_08, &charger_regs->reg08);
	status |= charger_read_register(BQ2560X_REG_09, &charger_regs->reg09);
	status |= charger_read_register(BQ2560X_REG_0A, &charger_regs->reg0A);
	status |= charger_read_register(BQ2560X_REG_0B, &charger_regs->reg0B);
	return status;
}

int8_t charger_read_status_regs(charger_regs_t *regs){
	int8_t status = 0;
	status |= charger_read_register(BQ2560X_REG_08, &charger_regs->reg08);
	status |= charger_read_register(BQ2560X_REG_09, &charger_regs->reg09);
	status |= charger_read_register(BQ2560X_REG_0A, &charger_regs->reg0A);
	return status;
}

int8_t charger_parse_status(charger_regs_t *regs, charger_t *charger) {

	if (regs->reg0A & REG0A_VBUS_GD_MASK) {
		charger->vbus_attached = 1;
	} else {
		charger->vbus_attached = 0;
	}

	if (regs->reg08 & REG08_PG_STAT_MASK) {
		charger->power_good = 1;
	} else {
		charger->power_good = 0;
	}

	charger -> charge_status = ((regs->reg08 & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT);
	charger -> vbus_status = ((regs->reg08 & REG08_VBUS_STAT_MASK) >> REG08_VBUS_STAT_SHIFT);

}

int8_t charger_read_register(uint8_t reg_addr, uint8_t *buf) {
	uint8_t txbuff[1];
	uint8_t rxbuff[1];
	msg_t status;
	txbuff[0] = reg_addr;
	i2cAcquireBus(&CHARGER_IF);
	status = i2cMasterTransmitTimeout(&CHARGER_IF, CHARGER_ADDRESS, txbuff, 1,
			rxbuff, 1, 1000);
	i2cReleaseBus(&CHARGER_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened withs charger: status is %d\r\n", i2cGetErrors(&CHARGER_IF));
		chSemSignal(&usart1_semaph);
		return -1;
	}
	*buf = rxbuff[0];
	return 0;
	/*chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Readed from charger %d\r\n",
			rxbuff[0]);
	chSemSignal(&usart1_semaph);*/
}

int8_t charger_write_register(uint8_t reg_addr, uint8_t *txbuf, uint8_t txbytes) {

	uint8_t i = 0;
	uint8_t buff[32];
	msg_t status;
	memset(buff, 0, 32);
	buff[0] = reg_addr;
	memcpy(&buff[1], txbuf, txbytes);

	i2cAcquireBus(&CHARGER_IF);
	status = i2cMasterTransmitTimeout(&CHARGER_IF, CHARGER_ADDRESS, buff, txbytes,
			NULL, 0, 1000);
	i2cReleaseBus(&CHARGER_IF);
	if (status != MSG_OK) {
		chSemWait(&usart1_semaph);
		chprintf((BaseSequentialStream*) &SD1,
				"Shit happened with writing charger: status is %d\r\n", i2cGetErrors(&CHARGER_IF));
		chSemSignal(&usart1_semaph);
		return -1;
	}
	return 0;
	/*chSemWait(&usart1_semaph);
	chprintf((BaseSequentialStream*) &SD1, "Readed from charger %d\r\n",
			rxbuff[0]);
	chSemSignal(&usart1_semaph);*/
}

void start_charger_module(void){
	chThdCreateStatic(charger_thread_wa, sizeof(charger_thread_wa), NORMALPRIO, charger_thread, NULL);
}
