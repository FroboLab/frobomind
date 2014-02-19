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
# Created:  2014-02-17
****************************************************************************/
/* includes */
#include <avr/interrupt.h>
#include <stdlib.h>
#include "rcdef.h"
#include "avr_serial.h"

/***************************************************************************/
/* defines */

#define false				0
#define true				1

/* defines for the timer1 interrupt */
#define INT1_CNT			200 /* 100ns */
#define FLIPBIT				PC3
#define FLIPBIT_PORT		PORTC
#define FLIPBIT_DDR			DDRC

/* signal led defines */
#define LED_STATE_OFF			0
#define LED_STATE_ON			1
#define LED_DELAY			4 /* times the cycle */

/***************************************************************************/
/* global and static variables */

/* timer1 and scheduler variables */
volatile unsigned char sched;
unsigned short sched_cnt;

/* system variables */
char state;

/* user interface variables */
char led_state;
char led_signal;
char led_count;

/* serial interface variables */
unsigned char c;
unsigned short ms_since_last, ms_since_last_tmp;

/***************************************************************************/
void sched_init(void)
{
	/* timer 1 interrupt init */
	sched = 0;
	sched_cnt = 0;
    TIMSK1 = BV(OCIE1A); 
    TCCR1B = BV(CS11) | BV(WGM12); /* clk/8, Clear Timer on Compare Match (OCR1A) */  
    OCR1A = INT1_CNT;
	PB_OUT (FLIPBIT_DDR, FLIPBIT); /* set flipbit as output */
}
/***************************************************************************/
/*ISR(SIG_OUTPUT_COMPARE1A) */
ISR (TIMER1_COMPA_vect)
{
	sched++;
	PB_FLIP (FLIPBIT_PORT, FLIPBIT); /* time to flip the flip bit */
}
/***************************************************************************/
void sched_update (void)
{
	sched_cnt++;
	if (sched_cnt == 10000)
		sched_cnt = 0;
	ms_since_last++;

	if (sched_cnt % 5000 == 0) /* each 100 ms */
	{
		RC_LED_FLIP;
	}
}
/***************************************************************************/
int main(void)
{
	sched_init(); /* initialize the scheduler */
	RC_LED_INIT; /* initialize RoboCard LED */
	serial_init(); /* initialize serial communication */
	ms_since_last = 0;	

	sei(); /* enable interrupts */
	for (;;) /* go into an endless loop */
	{
		/* motor_update(); */

		if (sched != 0) /* if the interrupt has timed out after 1 ms */
		{
			sched --;
			sched_update(); /* run the scheduler */
		}
		else
		{
			if (serial_rx_avail())
			{
				c = serial_rx();
				if ( c == '$')
				{	
					ms_since_last_tmp = ms_since_last;
					ms_since_last = 0;
					
					if (ms_since_last_tmp <= 250)
						c = ms_since_last_tmp;
					else
						c = 251;
					serial_tx (c);
				}
			}
		}
	}
	return 0; /* just for the principle as we never get here */
}
/***************************************************************************/
