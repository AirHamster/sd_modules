/*
 * pwr_mgmt_l4.c
 *
 *  Created on: Nov 27, 2019
 *      Author: a-h
 */


#include "config.h"
#include <hal.h>
#include "math.h"
#include <shell.h>
#include <string.h>
#include <stdlib.h>
#include "pwr_mgmt_l4.h"


void enter_stop_mode(void);
void leave_stop_mode(void);
void deactivate_clocks(void);
void activate_clocks(void);

static void pwr_pins_to_save_state(void);
static void pwr_pins_to_work_state(void);
static volatile uint8_t device_power_state = 0;
static volatile uint8_t deepsleep = 1;

static event_source_t button_pressed_event;
static event_source_t button_released_event;
static event_source_t charger_int_low_event;
static event_source_t charger_int_high_event;
extern event_source_t power_state_change_event;
static void button_cb(void *arg) {

	(void) arg;

	chSysLockFromISR();
if (deepsleep == 1) {
	//activate_clocks();
	pwr_pins_to_work_state();
	deepsleep = 0;
	}
	if (palReadLine(LINE_POWER_BUTTON) == PAL_HIGH) {
		chEvtBroadcastI(&button_pressed_event);
	} else {
		chEvtBroadcastI(&button_released_event);
	}
	chSysUnlockFromISR();
	//chprintf(SHELL_IFACE, "Event!");
}

static void charger_int_cb(void *arg) {
	(void) arg;
	chSysLockFromISR();
	if (deepsleep == 1) {
		activate_clocks();
		pwr_pins_to_work_state();
		deepsleep = 0;
	}
	if (palReadLine(LINE_CHARGER_INT) == PAL_LOW) {
		chEvtBroadcastI(&charger_int_low_event);
	} else {
		chEvtBroadcastI(&charger_int_high_event);
	}
	chSysUnlockFromISR();
	//chprintf(SHELL_IFACE, "Event!");

}

static THD_WORKING_AREA(pwr_mgmt_thread_wa, 512);
static THD_FUNCTION( pwr_mgmt_thread, p) {
	//(void) arg;

		chRegSetThreadName("Pwr butt");
		event_listener_t el0, el1;
		/* Events initialization and registration.*/
		chEvtObjectInit(&button_pressed_event);
		chEvtObjectInit(&button_released_event);
		chEvtRegister(&button_pressed_event, &el0, 0);
		chEvtRegister(&button_released_event, &el1, 1);



		/* Enabling events on both edges of the button line.*/
		palEnableLineEvent(LINE_POWER_BUTTON, PAL_EVENT_MODE_RISING_EDGE);
		palSetLineCallback(LINE_POWER_BUTTON, button_cb, NULL);

		while (true) {

			eventmask_t events;
			events = chEvtWaitOne(EVENT_MASK(0) | EVENT_MASK(1));
			if (events & EVENT_MASK(0)) {	//event for pressing
				chThdSleepMilliseconds(2000);
				if (palReadLine(LINE_POWER_BUTTON) == PAL_HIGH) {

					if (device_power_state == 0) {
						palSetLine(LINE_GREEN_LED);
						palSetLine(LINE_3_3_EN);
						palSetLine(LINE_5_EN);
						device_power_state = 1;
						chEvtBroadcastI(&power_state_change_event);
						//waking up device
					} else {
						palClearLine(LINE_GREEN_LED);
						palClearLine(LINE_3_3_EN);
						palClearLine(LINE_5_EN);
						device_power_state = 0;
						deepsleep = 1;
						//chEvtBroadcastI(&power_state_change_event);
						//put device into deep sleep
					}

				} else {
					deepsleep = 1;
				}
			}
			if (events & EVENT_MASK(1)) {	//event for releasing

			//	chThdSleepMilliseconds(2000);
				/*
				 if (palReadLine(LINE_POWER_BUTTON) == PAL_LOW) {
				 palClearLine(LINE_RED_LED);
				 device_power_state = 0;
				 }
				 */
			}
		}
}

void start_power_management_module(void){
	chThdCreateStatic(pwr_mgmt_thread_wa, sizeof(pwr_mgmt_thread_wa), NORMALPRIO, pwr_mgmt_thread, NULL);
}

void enter_stop_mode(void) {
if (deepsleep == 1){
	//deactivate_clocks();
	pwr_pins_to_save_state();
}

}

void leave_stop_mode(void) {


}

void activate_clocks(void){
	//switch off HSI16
		RCC->CR |= RCC_CR_HSION;
		while ((RCC->CR & RCC_CR_HSIRDY) == 0)
			; /* Wait until HSI16 is stable.  */
		/* Switching to the configured SYSCLK source if it is different from MSI.*/
		RCC->CFGR |= STM32_SW; /* Switches on the selected clock source.   */
		/* Wait until SYSCLK is stable.*/
		while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
			;

		/* PLL activation.*/
		RCC->CR |= RCC_CR_PLLON;

		/* Waiting for PLL lock.*/
		while ((RCC->CR & RCC_CR_PLLRDY) == 0)
			;

		RCC->CR |= RCC_CR_PLLSAI1ON;

		/* Waiting for PLL lock.*/
		while ((RCC->CR & RCC_CR_PLLSAI1RDY) == 0)
			;
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		/* HSI activation.*/
		//  RCC->CR |= RCC_CR_HSION;
		//  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                 /* Wait until HSI16 is stable.  */

		/*
		 SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		 stm32_clock_init();
		*/

		//sdStart(&SHELL_SD, NULL);
		//i2cStart(&CHARGER_IF, &charger_if_cfg);
}

void deactivate_clocks(void){

	// change main clock source to MSI
	// Clocking from MSI, in case MSI was not the default source.
	//	RCC->CFGR = 0;
	//	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);

	/* Switching to the configured SYSCLK source if it is different from MSI.*/
	RCC->CFGR = 0;
	RCC->CFGR |= STM32_SW_MSI; /* Switches on the selected clock source.   */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);
	//switch off HSI16
	RCC->CR &= ~RCC_CR_HSION;

	/* Wait until SYSCLK is stable.*/
//	while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW_MSI << 2));

	// PLLSAI
	RCC->CR &= ~RCC_CR_PLLSAI1ON;

	// PLL deactivation.
	RCC->CR &= ~RCC_CR_PLLON;

	//deepsleep and sleeponexit
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	//stop2 mode
	PWR->CR1 |= PWR_CR1_LPMS_STOP2;

	//palToggleLine(LINE_GREEN_LED);
	/*
	 // switch off all active peripherals

	 i2cStop(&CHARGER_IF);
	 sdStop(&SHELL_SD);

	 // PLLSAI2 deactivation.
	 RCC->CR |= RCC_CR_PLLSAI1ON;

	 // PLL deactivation.
	 RCC->CR &= ~RCC_CR_PLLON;
	 */
}

static void pwr_pins_to_save_state(void){

/*
	palSetLineMode(LINE_USART1_TX, PAL_MODE_INPUT_ANALOG);
	palSetLineMode(LINE_USART1_RX, PAL_MODE_INPUT_ANALOG);

	palSetLineMode(LINE_USART2_TX, PAL_MODE_INPUT_ANALOG);
	palSetLineMode(LINE_USART2_RX, PAL_MODE_INPUT_ANALOG);
*/
	//TODO: SPI pins for flash memory
		/*
		palSetLineMode(LINE_USART2_TX, PAL_MODE_ALTERNATE(4));
		palSetLineMode(LINE_USART2_RX, PAL_MODE_ALTERNATE(4));
		palSetLineMode(LINE_USART2_TX, PAL_MODE_ALTERNATE(4));
		palSetLineMode(LINE_USART2_RX, PAL_MODE_ALTERNATE(4));
	*/

}

static void pwr_pins_to_work_state(void){

	/*
	palSetLineMode(LINE_USART1_TX, PAL_MODE_ALTERNATE(4));
	palSetLineMode(LINE_USART1_RX, PAL_MODE_ALTERNATE(4));

	palSetLineMode(LINE_USART2_TX, PAL_MODE_ALTERNATE(4));
	palSetLineMode(LINE_USART2_RX, PAL_MODE_ALTERNATE(4));
*/
	//TODO: SPI pins for flash memory
	/*
	palSetLineMode(LINE_USART2_TX, PAL_MODE_ALTERNATE(4));
	palSetLineMode(LINE_USART2_RX, PAL_MODE_ALTERNATE(4));
	palSetLineMode(LINE_USART2_TX, PAL_MODE_ALTERNATE(4));
	palSetLineMode(LINE_USART2_RX, PAL_MODE_ALTERNATE(4));
*/

}

static void pwr_terminate_threads(){


}

static void pwr_restart_threads(){

}

void pwr_switch_dc_dc(dcdc_enum dcdc, dcdc_enum state){

}

void pwr_start_threads(void){

}

void pwr_stop_threads(void){

}
