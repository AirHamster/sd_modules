/*
 * commsthread.c
 *
 *  Created on: 2 Sep 2014
 *      Author: shd
 */

#include <string.h>

#include <hal.h>


#include "i2cslave.h"

#include "chprintf.h"
#include "memstreams.h"


/**
 * I2C slave test routine.
 *
 * To use: Add file to a project, call startComms() with the address of a serial stream
 *
 * There are two different responses:
 *  a) A read-only transaction - returns the "Initial Reply" message
 *  b) A write then read transaction - calls a message processor and returns the generated reply.
 *          Stretches clock until reply available.
 */


#define slaveI2cPort    I2CD1
#define slaveI2Caddress  0x30       /* Address in our terms - halved by later code */
//#define myOtherI2Caddress 0x19


/** Event flags */
#define I2C_ERROR_EVENT     1
#define I2C_POLL_EVENT      2
#define I2C_TXRX_EVENT      4

thread_t *I2CMonThread = NULL;      /* Thread to monitor I2C events */


I2CSlaveMsgCB messageProcessor, catchError, clearAfterSend, notePoll;

/* I2C1 configuration */
#ifdef STM32F0xx_MCUCONF
static const I2CConfig slaveI2Cconfig = {
//    0x10420f13,     // clock timing - 100kHz with 8MHz clock
     STM32_TIMINGR_PRESC(5U)  |            /* 48MHz/6 = 8MHz I2CCLK.           */
     STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
     STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
  //0x00310309,     // Hopefully 400kHz
                    // Just scale bits 28..31 if not 8MHz clock
    0,              // CR1
    0,              // CR2
    NULL            // Callback
};

#define I2C_PIN_MODE        1
#endif

#ifdef STM32F7xx_MCUCONF
// Following numbers trimmed to give 100kHz clock - waveform reasonably symmetrical
static const I2CConfig slaveI2Cconfig = {
     STM32_TIMINGR_PRESC(10U)  |
     STM32_TIMINGR_SCLDEL(9U) | STM32_TIMINGR_SDADEL(9U) |
     STM32_TIMINGR_SCLH(21U)   | STM32_TIMINGR_SCLL(24U),
    0,              // CR1
    0,              // CR2
    NULL            // Receive callback
};
#define I2C_PIN_MODE        4
#endif


#ifdef STM32F4xx_MCUCONF
static const I2CConfig slaveI2Cconfig = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE
};
#endif

//char initialReplyBody[50] = "Initial reply";        // 'Status' response if read without preceding write
char initialReplyBody[550] =
    "----- Initial reply ------"
    "A3012345678901234567890xxx1234567890123456789012345\r\n"
    "B3012345678901234567890xxx1234567890123456789012345\r\n"
    "C3012345678901234567890xxx1234567890123456789012345\r\n"
    "D3012345678901234567890xxx1234567890123456789012345\r\n"
    "E3012345678901234567890xxx1234567890123456789012345\r\n"
    ;


uint32_t messageCounter = 0;                /* Counts number of messages received to return as part of response */

uint8_t  rxBody[600];                       /* stores last message master sent us (intentionally a few bytes smaller than txBody) */
uint8_t  txBody[600];                       /* Return message buffer for computed replies */

BaseSequentialStream *chp = NULL;           // Used for serial logging

// Handler when something sent to us
const I2CSlaveMsg echoRx =
{
  sizeof(rxBody),       /* max sizeof received msg body */
  rxBody,               /* body of received msg */
  NULL,                 /* do nothing on address match */
  messageProcessor,     /* Routine to process received messages */
  catchError            /* Error hook */
};


// 'Empty' reply when nothing to say, and no message received. In RAM, to allow update
I2CSlaveMsg initialReply =
{
  sizeof(initialReplyBody),  /* trailing zero byte will be repeated as needed */
  (uint8_t *)initialReplyBody,
  NULL,                 /* do nothing on address match */
  notePoll,             /* Just register an event */
  catchError            /* Error hook */
};


// Response to received messages
I2CSlaveMsg echoReply = {  /* this is in RAM so size may be updated */
  0,                    /* filled in with the length of the message to send */
  txBody,               /* Response message */
  NULL,                 /* do nothing special on address match */
  clearAfterSend,       /* Clear receive buffer once replied */
  catchError            /* Error hook */
};


/**
 * Track I2C errors
 */
uint32_t lastI2cErrorFlags = 0;


/**
 * Generic error handler
 *
 * Called in interrupt context, so need to watch what we do
 */
void catchError(I2CDriver *i2cp)
{
  lastI2cErrorFlags = i2cp->errors;
  chSysLockFromISR();
  chEvtSignalI(I2CMonThread, I2C_ERROR_EVENT);       // Trigger to print the error
  chSysUnlockFromISR();
}


/**
 * Note that we were polled (read request without preceding write)
 *
 * Called in interrupt context, so need to watch what we do
 */
void notePoll(I2CDriver *i2cp)
{
  (void) i2cp;
  chSysLockFromISR();
  chEvtSignalI(I2CMonThread, I2C_POLL_EVENT);
  chSysUnlockFromISR();
}



const char hexString[16] = "0123456789abcdef";


/**
 *  Message processor - looks at received message, determines reply as quickly as possible
 *
 *  Responds with the value of the messageCounter (in hex), followed by the received message in [..]
 *
 *  Note: Called in interrupt context, so need to be quick!
 */
void messageProcessor(I2CDriver *i2cp)
{
  uint8_t i;
  uint8_t *txPtr = txBody + 8;
  size_t txLen;
  uint32_t curCount;

  size_t len = i2cSlaveBytes(i2cp);         // Number of bytes received
  if (len >= sizeof(rxBody))
      len = sizeof(rxBody)-1;
  rxBody[len]=0;                            // String termination sometimes useful

  /* A real-world application would read and decode the message in rxBody, then generate an appropriate reply in txBody */

  curCount = ++messageCounter;
  txLen = len + 11;                         // Add in the overhead

  for (i = 0; i < 8; i++)
  {
    *--txPtr = hexString[curCount & 0xf];
    curCount = curCount >> 4;
  }

  txPtr = txBody + 8;
  *txPtr++ = ' ';
  *txPtr++ = '[';
  memcpy(txPtr, rxBody, len);               // Echo received message
  txPtr += len;
  *txPtr++ = ']';
  *txPtr = '\0';

  /** Message ready to go here */
  echoReply.size = txLen;
  chSysLockFromISR();
  i2cSlaveReplyI(i2cp, &echoReply);
  chEvtSignalI(I2CMonThread, I2C_TXRX_EVENT);
  chSysUnlockFromISR();
}



/**
 * Callback after sending of response complete - restores default reply in case polled
 */
void clearAfterSend(I2CDriver *i2cp)
{
  echoReply.size = 0;               // Clear receive message
  i2cSlaveReplyI(i2cp, &initialReply);
}







/**
 * Thread to monitor I2C events and print messages on the console
 */

static THD_WORKING_AREA(waI2CMonThread1, 256);
__attribute__((noreturn)) static THD_FUNCTION(I2CMonThread1, arg)
{
  (void)arg;
  eventmask_t newEvents;

  chRegSetThreadName("I2C Monitor");

  while (TRUE)
  {
    newEvents = chEvtWaitAny(
        I2C_ERROR_EVENT |
        I2C_POLL_EVENT |
        I2C_TXRX_EVENT
        );

    if (newEvents & I2C_ERROR_EVENT)
    {
      chprintf(chp, "I2cError: %04x\r\n", lastI2cErrorFlags);
    }
    if (newEvents & I2C_POLL_EVENT)
    {
      chprintf(chp, "I2c Poll\r\n");
    }
    if (newEvents & I2C_TXRX_EVENT)
    {
      chprintf(chp, "I2c TxRx\r\n");
    }
    chThdSleepMilliseconds(1);
  }
}


/**
 * Start the I2C Slave port to accept comms from master CPU
 *
 * We then go into a loop checking for errors, and never return
 */
void startComms(BaseSequentialStream *serport)
{
  chp = serport;

  palSetPadMode(GPIOB, 8, PAL_MODE_INPUT);       // Try releasing special pins for a short time
  palSetPadMode(GPIOB, 9, PAL_MODE_INPUT);       // Try releasing special pins for a short time
  chThdSleepMilliseconds(10);

  /* I2C1 SCL on PF1, SDA on PF0 */
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(I2C_PIN_MODE) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUDR_PULLUP);
  palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(I2C_PIN_MODE) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUDR_PULLUP);


  i2cStart(&slaveI2cPort, &slaveI2Cconfig);
#if HAL_USE_I2C_SLAVE
  slaveI2cPort.slaveTimeout = MS2ST(500);       // Time for complete message
#endif

  initialReply.size = strlen((char *)initialReply.body) + 1;                // Set the initial return message length
  i2cSlaveConfigure(&slaveI2cPort, &echoRx, &initialReply);

  // Enable match address after everything else set up
  i2cMatchAddress(&slaveI2cPort, slaveI2Caddress/2);
//  i2cMatchAddress(&slaveI2cPort, myOtherI2Caddress/2);
//  i2cMatchAddress(&slaveI2cPort, 0);  /* "all call" */

  chprintf(chp, "Slave I2C started\n\r");

  I2CMonThread = chThdCreateStatic(waI2CMonThread1, sizeof(waI2CMonThread1), NORMALPRIO, I2CMonThread1, NULL);

}




