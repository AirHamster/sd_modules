/*
 * nina-b3.c
 *
 *  Created on: Apr 26, 2019
 *      Author: a-h
 */

#include "nina-b3.h"

void nina_get_discoverable_status(void){
	chprintf((BaseSequentialStream*)&SD7, "AT+UBTDM?");
}

