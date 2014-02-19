/****************************************************************************
# RoboCard Defines (RCdef)
# Copyright (c) 2013, Kjeld Jensen <kj@kjen.dk>
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
# File: rcdef.h
# Project: RoboCard Defines (RCdef)
# Platform: RoboCard v1.11 http://www.robocard.org
# Microcontroller: ATmega168p
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2013-02-06
# Modified: 2014-02-17 Added RC_LED_FLIP
****************************************************************************/

#ifndef _RCDEF_H /* only define the below once */
#define _RCDEF_H

/* includes */
#include <avr/io.h>

/* general defines */
#define FALSE				0
#define TRUE				1

#define BV(bit)				(1<<bit) /* return bit value for a bit */

/* ATmega port defines (output) */
#define PB_OUT( ddr, bit)		ddr |= BV(bit) /* set port bit as output */
#define PB_HIGH( port, bit)		port |= BV(bit) /* set port bit high */
#define PB_LOW( port, bit)		port &= ~BV(bit) /* set port bit low */
#define PB_FLIP( port, bit)		port ^= BV(bit) /* flip port bit */

/* ATmega port defines (input) */
#define PB_IN( ddr, bit)		ddr &= ~BV(bit) /* set port bit as input */
#define PB_PULL_UP( port, bit)		PB_HIGH(port, bit) /* enable pull-up resistor */
#define PB_IS_HIGH( inport, bit)	inport & BV(bit) /* true if port bit is high */
#define PB_IS_LOW( inport, bit)		!(inport & BV(bit) /* true if port bit is low */

/* RoboCard LED defines */
#define RC_LED_INIT			PB_OUT (DDRB,DDB0) /* set LED bit as output */
#define RC_LED_ON			PB_LOW (PORTB,PB0) /* turn LED on */
#define RC_LED_OFF			PB_HIGH (PORTB,PB0) /* turn LED off */
#define RC_LED_FLIP			PB_FLIP (PORTB,PB0) /* flip LED */

/* RoboCard Boot Loader jumper defines */
#define RC_BL_INIT			PB_PULL_UP (PORTD, PD7) /* enable pull-up resistor */
#define RC_BL_SHORT			PB_IS_LOW (PIND,PD7) /* true if jumper is installed */
#define RC_BL_OPEN			PB_IS_HIGH (PIND,PD7) /* true if no jumper */

#endif /* this must be the last line! */
/***************************************************************************/

