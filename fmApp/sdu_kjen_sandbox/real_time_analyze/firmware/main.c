/****************************************************************************
# FroboMind real time analyze firmware
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
# File: main.c
# Project: FroboMind real time analyze firmware
# Platform: RoboCard v1.11 http://www.robocard.org
# Microcontroller: ATmega48
# Author: Kjeld Jensen <kjeld@frobomind.org>
# Created:  2014-02-21
****************************************************************************/
/* includes */
#include <avr/interrupt.h>
#include <stdlib.h>
#include "rcdef.h"
#include "avr_serial.h"

/***************************************************************************/
/* defines */

/* defines for the timer1 interrupt */
#define INT1_CNT			199 /* 100ns at 16 MHz with clock/8 divider */

/* LED defines */
#define FLIPBIT				PB1
#define FLIPBIT_PORT		PORTB
#define FLIPBIT_DDR			DDRB

/***************************************************************************/
/* global and static variables */

/* timer1 and scheduler variables */
volatile unsigned char sched_int;
unsigned short sched_cnt;

/* serial interface variables */
unsigned char c;
unsigned short cnt_since_last, cnt_since_last_tmp;

/***************************************************************************/
void sched_init(void)
{
	/* timer 1 interrupt init */
	sched_int = 0;
    TIMSK1 = BV(OCIE1A); 
    TCCR1B = BV(CS11) | BV(WGM12); /* clk/8, Clear Timer on Compare Match (OCR1A) */  
    OCR1A = INT1_CNT;
	PB_OUT (FLIPBIT_DDR, FLIPBIT); /* set flipbit as output */
}
/***************************************************************************/
/*ISR(SIG_OUTPUT_COMPARE1A) */
ISR (TIMER1_COMPA_vect)
{
	sched_int++;
	PB_FLIP (FLIPBIT_PORT, FLIPBIT); /* time to flip the flip bit */
}
/***************************************************************************/
int main(void)
{
	sched_init(); /* initialize the scheduler */
	RC_LED_INIT; /* initialize RoboCard LED */
	serial_init(); /* initialize serial communication */
	sched_cnt = 0;
	cnt_since_last = 0;	
	sei(); /* enable interrupts */

	for (;;) /* go into an endless loop */
	{
		if (sched_int != 0) /* if the scheduler interrupt has timed out */
		{
			sched_int--;
			cnt_since_last++;
			sched_cnt++;
			if (sched_cnt == 10000)
				sched_cnt = 0;

			if (sched_cnt % 5000 == 0) /* each 500 ms */
			{
				RC_LED_FLIP;
			}
		}
		if (serial_rx_avail())
		{
			c = serial_rx();
			if (c == '$')
			{	
				cnt_since_last_tmp = cnt_since_last;
				cnt_since_last = 0;
				
				if (cnt_since_last_tmp <= 250)
					c = cnt_since_last_tmp;
				else
					c = 251;
				serial_tx_direct (c);
			}
		}
	}
	return 0; /* just for the principle as we never get here */
}
/***************************************************************************/
