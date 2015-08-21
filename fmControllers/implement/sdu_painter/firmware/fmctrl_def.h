/****************************************************************************
# FroboMind Controller defines
# Copyright (c) 2014, Kjeld Jensen <kjen@mmmi.sdu.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name RoboCard nor the
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
# File: fmctl_def.h
# Project: FroboMind Controller
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Author: Kjeld Jensen <kjen@mmmi.sdu.dk>
# Created:  2014-01-09 Kjeld Jensen
# Modified: 2014-08-17 Kjeld Jensen, added define for 12V measurement
****************************************************************************/

#ifndef _FMCTL_H /* only define the below once */
#define _FMCTL_H

/* includes */
#include <avr/io.h>

/* general defines */
#define FALSE					0
#define TRUE					1

#define BV(bit)					(1<<bit) /* return bit value for a bit */

/* ATmega port defines (output) */
#define PB_OUT( ddr, bit)		ddr |= BV(bit) /* set port bit as output */
#define PB_HIGH( port, bit)		port |= BV(bit) /* set port bit high */
#define PB_LOW( port, bit)		port &= ~BV(bit) /* set port bit low */
#define PB_FLIP( port, bit)		port ^= BV(bit) /* flip port bit */

/* ATmega port defines (input) */
#define PB_IN( ddr, bit)		ddr &= ~BV(bit) /* set port bit as input */
#define PB_PULL_UP( port, bit)		PB_HIGH(port, bit) /* enable pull-up resistor */
#define PB_IS_HIGH( inport, bit)	inport & BV(bit) /* true if port bit is high */
#define PB_IS_LOW( inport, bit)		!(inport & BV(bit)) /* true if port bit is low */

/* LED defines */
#define INT_LED_INIT			PB_OUT (DDRE,DDE6) /* set LED bit as output */
#define INT_LED_ON				PB_LOW (PORTE,PE6) /* turn LED on */
#define INT_LED_OFF				PB_HIGH (PORTE,PE6) /* turn LED off */

#define EXT_LED_INIT			PB_OUT (DDRE,DDE5) /* set LED bit as output */
#define EXT_LED_ON				PB_LOW (PORTE,PE5) /* turn LED on */
#define EXT_LED_OFF				PB_HIGH (PORTE,PE5) /* turn LED off */

/* switch defines */
#define SWITCH_INIT				PB_PULL_UP( PORTA, PA6)
#define SWITCH_1_IS_ON			PB_IS_LOW( PINA, PINA6) /* true if switch 1 is enbled */
#define SWITCH_2_IS_ON			PB_IS_LOW( PINA, PINA7) /* true if switch 2 is enbled */

/* Vin measurement defines */
#define ADC_PORT_VOLT			0 /* ADC conn. to Vin through 22k/3.3k div. */
/* Voltage: ADC/1023 * 5 * (22000+3300)/3300  <=>  ADC*0.03747 */
#define VOLTAGE_12_0			320


#endif /* this must be the last line! */
/***************************************************************************/

