// UART.C
//
// Generic software uart written in C, requiring a timer set to 3 times
// the baud rate, and two software read/write pins for the receive and
// transmit functions.
//
// * Received characters are buffered
// * putchar(), getchar(), kbhit() and flush_input_buffer() are available
// * There is a facility for background processing while waiting for input
//
// Colin Gittins, Software Engineer, Halliburton Energy Services
//
// The baud rate can be configured by changing the BAUD_RATE macro as
// follows:
//
// #define BAUD_RATE			19200.0
//
// The function init_uart() must be called before any comms can take place
//
// Interface routines required:
// 1. get_rx_pin_status()
//    Returns 0 or 1 dependent on whether the receive pin is high or low.
// 2. set_tx_pin_high()
//    Sets the transmit pin to the high state.
// 3. set_tx_pin_low()
//    Sets the transmit pin to the low state.
// 4. idle()
//    Background functions to execute while waiting for input.
// 5. timer_set( BAUD_RATE )
//    Sets the timer to 3 times the baud rate.
// 6. set_timer_interrupt( timer_isr )
//    Enables the timer interrupt.
//
// Functions provided:
// 1. void flush_input_buffer( void )
//    Clears the contents of the input buffer.
// 2. char kbhit( void )
//    Tests whether an input character has been received.
// 3. char getchar( void )
//    Reads a character from the input buffer, waiting if necessary.
// 4. void turn_rx_on( void )
//    Turns on the receive function.
// 5. void turn_rx_off( void )
//    Turns off the receive function.
// 6. void putchar( char )
//    Writes a character to the serial port.


//17.05.2013


#include <stdio.h>
#include "software_uart.h"
/*
 * GPT4 configuration. This timer is used as trigger for the ADC.
 */
static GPTConfig gpt6cfg1;
#define BAUD_RATE		9600

#define IN_BUF_SIZE		255

#define TRUE 1
#define FALSE 0

static unsigned char		inbuf[IN_BUF_SIZE];
static unsigned char		qin = 0;
static unsigned char		qout = 0;

static char 			flag_rx_waiting_for_stop_bit;
static char 			flag_rx_off;
static char 			rx_mask;
static char 			flag_rx_ready;
static char 			flag_tx_ready;
static char 			timer_rx_ctr;
static char 			timer_tx_ctr;
static char 			bits_left_in_rx;
static char 			bits_left_in_tx;
static char 			rx_num_of_bits;
static char 			tx_num_of_bits;
static unsigned int		internal_rx_buffer;
static unsigned int		internal_tx_buffer;
static char          		user_tx_buffer;

static void susart_rx_cb(void *arg);
static int8_t get_rx_pin_status(void);
static void set_tx_pin_high(void);
static void set_tx_pin_low(void);
static void timer_set(uint16_t br);
static void set_timer_interrupt (void);
static void clear_timer_interrupt (void);
static void idle(void);

void timer3_isr_cb(GPTDriver *gptp)
	{
	(void)gptp;
	char			mask, start_bit, flag_in;
	//palToggleLine(LINE_RED_LED);
	//chprintf((BaseSequentialStream*) &SD1, "C");
	// Transmitter Section
	if ( flag_tx_ready )
		{
		if ( --timer_tx_ctr == 0 )
			{
			mask = internal_tx_buffer&1;
			internal_tx_buffer >>= 1;
			if ( mask )
				{
				set_tx_pin_high();
				}
			else
				{
				set_tx_pin_low();
				}
			timer_tx_ctr = 3;
			if ( --bits_left_in_tx<=0 )
				{
				flag_tx_ready = FALSE;
				clear_timer_interrupt();
				}
			}
		}
// Receiver Section
	if ( flag_rx_off==FALSE )
		{
		if ( flag_rx_waiting_for_stop_bit )
			{
			if ( --timer_rx_ctr ==0 )
				{
				flag_rx_waiting_for_stop_bit = FALSE;
				flag_rx_ready = FALSE;
				internal_rx_buffer &= 0xFF;
				if ( internal_rx_buffer!=0xC2 )
					{
					inbuf[qin] = internal_rx_buffer;
					if ( ++qin>=IN_BUF_SIZE )
						{
						qin = 0;
						}
					}
				}
			}
		else		// rx_test_busy
			{
			if ( flag_rx_ready==FALSE )
				{
				start_bit = get_rx_pin_status();
// Test for Start Bit
				if ( start_bit==0 )
					{
					flag_rx_ready = TRUE;
					internal_rx_buffer = 0;
					timer_rx_ctr = 4;
					bits_left_in_rx = rx_num_of_bits;
					rx_mask = 1;
					}
				}
			else	// rx_busy
				{
				if ( --timer_rx_ctr == 0 )
					{				// rcv
					timer_rx_ctr = 3;
					flag_in = get_rx_pin_status();
					if ( flag_in )
						{
						internal_rx_buffer |= rx_mask;
						}
					rx_mask <<= 1;
					if ( --bits_left_in_rx == 0 )
						{
						flag_rx_waiting_for_stop_bit = TRUE;
						}
					}
				}
			}
		}
	}

void susart_init( void )
	{
	flag_tx_ready = FALSE;
	flag_rx_ready = FALSE;
	flag_rx_waiting_for_stop_bit = FALSE;
	flag_rx_off = TRUE;
	rx_num_of_bits = 10;
	tx_num_of_bits = 10;

	set_tx_pin_high();

	/* Enabling events on both edges of the rx line.*/
//		palEnableLineEvent(LINE_SUSART1_RX, PAL_EVENT_MODE_BOTH_EDGES);
	//	palSetLineCallback(LINE_SUSART1_RX, susart_rx_cb, NULL);

	timer_set( BAUD_RATE );
//	set_timer_interrupt(); 	// Enable timer interrupt
	}

char _getchar( void )
	{
	char		ch;

	do
		{
		while ( qout==qin )
			{
			idle();
			}
		ch = inbuf[qout] & 0xFF;
		if ( ++qout>=IN_BUF_SIZE )
			{
			qout = 0;
			}
		}
	while ( ch==0x0A || ch==0xC2 );
	return( ch );
	}

void _putchar( char ch )
	{
	while ( flag_tx_ready );


	user_tx_buffer = ch;

// invoke_UART_transmit
	timer_tx_ctr = 3;
	bits_left_in_tx = tx_num_of_bits;
	internal_tx_buffer = (user_tx_buffer<<1) | 0x200;
	flag_tx_ready = TRUE;
	set_timer_interrupt();
	}

void flush_input_buffer( void )
	{
	qin = 0;
	qout = 0;
	}

char kbhit( void )
	{
	return( qin!=qout );
	}

void turn_rx_on( void )
	{
	flag_rx_off = FALSE;
	}

void turn_rx_off( void )
	{
	flag_rx_off = TRUE;
	}

static void susart_rx_cb(void *arg){
	(void) arg;
}
// platform specific funcs
static int8_t get_rx_pin_status(void){
	return palReadLine(LINE_SUSART1_RX);
}

static void set_tx_pin_high(void){
	palSetLine(LINE_SUSART1_TX);
}

static void set_tx_pin_low(void){
	palClearLine(LINE_SUSART1_TX);
}

static void timer_set(uint16_t br){
	 gpt6cfg1.frequency =  br *3 * 100;
	 gpt6cfg1.callback  =  timer3_isr_cb;
	 gpt6cfg1.cr2       =  0U;
	 gpt6cfg1.dier      =  0U;
	 gptStart(&GPTD6, &gpt6cfg1);
	 set_timer_interrupt();
	 gptStartContinuous(&GPTD6, 100);
}

static void set_timer_interrupt (void){
	TIM6->DIER      =  TIM_DIER_UIE;
}

static void clear_timer_interrupt (void){
	TIM6->DIER      =  0U;
}

static void idle(void){

}
