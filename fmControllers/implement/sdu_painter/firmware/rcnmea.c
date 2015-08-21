/****************************************************************************
# RoboCard NMEA Communication Library
# Copyright (c) 2012-2013, Kjeld Jensen <kj@kjen.dk>
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
# File: rcnmea.c
# Project: RoboCard Serial Communication Library
# Platform: RoboCard v1.11 http://www.robocard.org
# Microcontroller: ATmega88PA
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2012-08-29 Kjeld Jensen
# Modified: 2012-12-13 Kjeld Jensen, renamed nmea_tx_append() and added nmea_tx_append_ushort()
# Modified: 2013-02-04 Kjeld Jensen, moved to the BSD license
# Modified: 2014-04-17 Kjeld Jensen, switched to an updated serial driver
# Modified: 2014-08-21 Kjeld Jensen, added serial tx of 13,10 after reset
****************************************************************************/
/* includes */

#include <stdlib.h>
#include "avr_serial.h"
#include "rcnmea.h"
/***************************************************************************/
/* global and static variables */
char rx[RXBUF_SIZE];
char tx[TXBUF_SIZE];
short rx_len;
short rx_ite;
short tx_len;
short nmea_err;

/***************************************************************************/
void nmea_reset(void)
{
	rx_len = -1; /* wait for next $ */
	nmea_err = 0; /* reset serial error flag */
	serial_tx (13);		
	serial_tx (10);		
}
/***************************************************************************/
static unsigned char hexval (char c)
{
	unsigned char v = 0;
	if (c <= '9')
		v = c-'0';
	else if (c <= 'F')
		v = c+10-'A';
	else if (c <= 'f')
		v = c+10-'a';
	return v;
}
/***************************************************************************/
static char nmea_rx_validate(void)
{
	char csok = 0;
	unsigned char cs_calc = 0;
	unsigned char cs_recv;
	short i;

	/* assume that $, CR and LF are not present the buffer */
	
	/* check if a checksum is present */
	if (rx_len >= 9 && rx[rx_len-3] == '*') /* contains at least "$XXXXX,*HH" */
	{
		/* calculate the checksum */
		for (i=0; i<rx_len-3; i++)
			cs_calc ^= rx[i];

		/* retrieve the checksum */
		cs_recv = hexval (rx[rx_len-2]) << 4 | hexval (rx[rx_len-1]);
		if (cs_calc == cs_recv)
			csok = 1;
	}
	/* no checksum */
	else if (rx_len >= 6) /* contains at least "$XXXXX," */
		csok = 1;
	return (csok);
}
/***************************************************************************/
short nmea_rx_next_val(void)
{
	short val = 0;
	if (rx[rx_ite] == ',' && rx[rx_ite+1] != ',')
	{
		short i = rx_ite+1;
		char tmp;
		while (i < rx_len && rx[i] != ',' && rx[i] != '*')
			i++;
		tmp = rx[i];
		rx[i] = 0;
		val = atoi (rx+rx_ite+1);
		rx[i] = tmp;
		if (tmp == ',')
			rx_ite = i;
		else
			rx_ite = -1;
	}
	return val;
}
/***************************************************************************/
void tx_inbuf(void)
{
	if (rx_len > 0)
	{
		short i;
		serial_tx ('$');		
		serial_tx ('E');		
		serial_tx ('E');		
		serial_tx ('E');		
		serial_tx ('E');		
		serial_tx ('E');		
		serial_tx (',');		
		for (i=0; i< rx_len; i++)
			serial_tx (rx[i]);		
		serial_tx (13);		
		serial_tx (10);		
	} 
}

void nmea_rx_update(void)
{
	/* update nmea buffer */
	while (serial_rx_avail())
	{
		char c = serial_rx();
		if (c=='$') { /* start of message */
			if (rx_len != -1) {
				nmea_err++;
			}
			rx_len = 0;
		}
		else if (c==CHAR_CR || c==CHAR_LF) /* end of message */
		{
			if (rx_len >= 4) {
				if (nmea_rx_validate())
					nmea_rx_parse();
				else
				{
					nmea_err++; 
/* tx_inbuf(); */
				}
			}
			rx_len = -1; /* waiting for next start of message */
		}
		else if (rx_len != -1) {
			/* copy anything but $ and CR LF to the buffer */
			rx[rx_len++] = c;
			if (rx_len == RXBUF_SIZE) { /* buffer overflow error */
				rx_len = -1; /* waiting for next start of message */
				nmea_err++;
			}
		}
	}
}
/***************************************************************************/
void nmea_tx_append_short(short val)
{
	itoa (val, tx+tx_len, 10);
	while (tx[tx_len] != 0)
		tx_len++;

	tx[tx_len++] = ',';
}
/***************************************************************************/
void nmea_tx_append_ushort(unsigned short val)
{
	itoa (val, tx+tx_len, 10);
	while (tx[tx_len] != 0)
		tx_len++;

	tx[tx_len++] = ',';
}
/***************************************************************************/
void nmea_tx (void)
{
	short i;
	unsigned char cs = 0;

	/* calculate checksum */
	tx[tx_len++] = '*';
	for (i=1; i<tx_len-1; i++)
		cs ^= tx[i];
	utoa(cs,tx+tx_len,16);

	/* force capital letters */
	i = tx_len+2;
	for (; tx_len!=i; tx_len++) {
		if(tx[tx_len] > 0x60 && tx[tx_len] < 0x67)
			tx[tx_len] -= 0x20;
	}

	/* add CR LF and send the message */
	tx[tx_len++] = 13;
	tx[tx_len++] = 10;
	tx[tx_len++] = 0;
	serial_tx_string(tx);
}
/***************************************************************************/
