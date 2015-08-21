/****************************************************************************
# RoboCard NMEA Communication Library
# Copyright (c) 2012-2014, Kjeld Jensen <kj@kjen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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
# File: rcnmea.h
# Project: RoboCard Serial Communication Library
# Platform: RoboCard v1.11 http://www.robocard.org
# Microcontroller: ATmega88PA
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2012-08-29 Kjeld Jensen
# Modified: 2012-12-13 Kjeld Jensen, renamed nmea_tx_append() and added nmea_tx_append_ushort()
# Modified: 2013-02-04 Kjeld Jensen, moved to the BSD license
****************************************************************************/

#ifndef _RCNMEA_H
#define _RCNMEA_H

/***************************************************************************/
/* defines */
#define RXBUF_SIZE			160
#define TXBUF_SIZE			160 
#define CHAR_CR				13
#define CHAR_LF				10

/***************************************************************************/
/* shared variables */

extern char rx[RXBUF_SIZE];
extern char tx[TXBUF_SIZE];
extern short rx_len;
extern short rx_ite;
extern short tx_len;
extern short nmea_err;

/***************************************************************************/
/* function prototypes */

void nmea_reset(void);
short nmea_rx_next_val(void);
void nmea_rx_update(void);
void nmea_tx_append_short(short val);
void nmea_tx_append_ushort(unsigned short val);
void nmea_tx (void);

void nmea_rx_parse(void); /* callback function */

/***************************************************************************/
#endif
