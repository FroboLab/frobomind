/****************************************************************************
# AVR Serial Communication Library
# Copyright (c) 2009-2014, Kjeld Jensen <kj@kjen.dk>
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
# File: avr_serial.c
# Project: AVR Serial Communication Library
# Platform: AVR
# Microcontroller: Atmel AVR
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2009-12-08 Kjeld Jensen
# Modified: 2011-02-21 Kjeld Jensen, released under the MIT license
# Modified  2011-03-14 Kjeld Jensen, added serial_rx_flush()
# Modified  2011-12-18 Kjeld Jensen, added serial_tx_idle
# Modified  2013-01-15 Kjeld Jensen, added support for double speed baud rates
# Modified  2013-02-04 Kjeld Jensen, migrated to the BSD license 
# Modified  2013-02-18 Kjeld Jensen, added tx interrupt routine
# Modified  2013-11-27 Kjeld Jensen, added support for easy port configuration
#                                    added tx interrupt routine
# Modified  2014-02-05 Kjeld Jensen, added configuration for port 1
# Modified  2014-02-17 Kjeld Jensen, added configuration for avr's with one port only
# Modified  2014-02-21 Kjeld Jensen, added serial_tx_direct()
****************************************************************************/
/* includes */

#include <avr/io.h>
#include <avr/interrupt.h>

/***************************************************************************/
/* parameters for serial communication */

#define FOSC 		16000000	/* oscillator frequency [Hz] */
#define BAUD 		57600		/* baud rate */
#define USART_1					/* serial port */
/* #define DOUBLE_SPEED_MODE */

/***************************************************************************/
/* defines */

#ifdef USART
	#define UCSRA_REG 	UCSR0A
	#define UCSRB_REG 	UCSR0B
	#define UCSRC_REG 	UCSR0C
	#define UBRRL_REG 	UBRR0L
	#define UBRRH_REG 	UBRR0H
	#define UDR_REG		UDR0
	#define RXEN_BIT	RXEN0
	#define TXEN_BIT	TXEN0
	#define RXCIE_BIT	RXCIE0
	#define TXCIE_BIT	TXCIE0
	#define RXC_BIT		RXC0
	#define TXC_BIT		TXC0
	#define U2X_BIT		U2X0
	#define UCSZ0_BIT	UCSZ00
	#define UCSZ1_BIT	UCSZ01
	#define UDRE_BIT	UDRE0
#endif

#ifdef USART_0
	#define UCSRA_REG 	UCSR0A
	#define UCSRB_REG 	UCSR0B
	#define UCSRC_REG 	UCSR0C
	#define UBRRL_REG 	UBRR0L
	#define UBRRH_REG 	UBRR0H
	#define UDR_REG		UDR0
	#define RXEN_BIT	RXEN0
	#define TXEN_BIT	TXEN0
	#define RXCIE_BIT	RXCIE0
	#define TXCIE_BIT	TXCIE0
	#define RXC_BIT		RXC0
	#define TXC_BIT		TXC0
	#define U2X_BIT		U2X0
	#define UCSZ0_BIT	UCSZ00
	#define UCSZ1_BIT	UCSZ01
	#define UDRE_BIT	UDRE0
#endif

#ifdef USART_1
	#define UCSRA_REG 	UCSR1A
	#define UCSRB_REG 	UCSR1B
	#define UCSRC_REG 	UCSR1C
	#define UBRRL_REG 	UBRR1L
	#define UBRRH_REG 	UBRR1H
	#define UDR_REG		UDR1
	#define RXEN_BIT	RXEN1
	#define TXEN_BIT	TXEN1
	#define RXCIE_BIT	RXCIE1
	#define TXCIE_BIT	TXCIE1
	#define RXC_BIT		RXC1
	#define TXC_BIT		TXC1
	#define U2X_BIT		U2X1
	#define UCSZ0_BIT	UCSZ10
	#define UCSZ1_BIT	UCSZ11
	#define UDRE_BIT	UDRE1
#endif

#ifdef DOUBLE_SPEED_MODE
	#define UBRR (FOSC/BAUD/8 - 1)
#else
	#define UBRR (FOSC/BAUD/16 - 1)
#endif

#define FALSE		0
#define TRUE		1

/***************************************************************************/
#define IB_MAX		80
#define OB_MAX		80

unsigned char ib[IB_MAX], ob[OB_MAX];
short ib_head, ib_tail, ob_head, ob_tail;
char tx_busy;

/***************************************************************************/
void serial_init(void)
{
	/* enable tx and rx */
	UCSRB_REG = (1<<TXEN_BIT)|(1<<RXEN_BIT);

	/* set baud rate */
	UBRRH_REG = (unsigned char) ((UBRR)>>8);
	UBRRL_REG = (unsigned char) (UBRR); /* remember the ()! */

	/* asynchronous 8N1 */
	UCSRC_REG = (1<<UCSZ0_BIT)|(1<<UCSZ1_BIT);
	
	/* enable double speed mode if #DOUBLE_SPEED_MODE is set */
#ifdef DOUBLE_SPEED_MODE
	UCSRA_REG |= U2X_BIT;
#endif

	/* init rx  */
	UCSRB_REG |= (1 << RXCIE_BIT);
	ib_head = 0;
	ib_tail = 0;

	/* init tx */
	ob_head = 0;
	ob_tail = 0;
	tx_busy = FALSE;
}
/***************************************************************************/
#ifdef USART
	ISR (USART_RX_vect)
#endif
#ifdef USART_0
	ISR (USART0_RX_vect)
#endif
#ifdef USART_1
	ISR (USART1_RX_vect)
#endif
{
	ib_head++;
	if (ib_head == IB_MAX)
		ib_head = 0;
	if (ib_head != ib_tail) /* do not add if buffer overrun */
		ib[ib_head] = UDR_REG;
}
/***************************************************************************/
#ifdef USART
	ISR (USART_TX_vect)
#endif
#ifdef USART_0
	ISR (USART0_TX_vect)
#endif
#ifdef USART_1
	ISR (USART1_TX_vect)
#endif
{
	if (ob_head != ob_tail) /* if buffer is not empty */
	{
		ob_tail++; /* increment the buffer tail */ 
		if (ob_tail == OB_MAX)
			ob_tail = 0;
		UDR_REG = ob[ob_tail]; /* send the char */
	}
	else
	{
		UCSRB_REG &= ~(1<<TXCIE_BIT); /* disable tx interrupt */
		tx_busy = FALSE;
	}
}
/***************************************************************************/
void serial_tx_init ()
{
	if (ob_head != ob_tail) /* if buffer is not empty */
	{
		tx_busy = TRUE;
		UCSRB_REG |= (1 << TXCIE_BIT); /* enable tx interrupt */

		ob_tail++; /* increment the buffer tail */ 
		if (ob_tail == OB_MAX)
			ob_tail = 0;
		UDR_REG = ob[ob_tail]; /* send the char */
	}
}
/***************************************************************************/
void serial_tx (unsigned char c)
{
	ob_head++;
	if (ob_head == OB_MAX)
		ob_head = 0;
	if (ob_head != ob_tail) /* do not add if buffer overrun */
		ob[ob_head] = c;

	if (tx_busy == FALSE)
		serial_tx_init();
}
/***************************************************************************/
void serial_tx_direct (unsigned char c)
{
	/* wait for an empty transmit buffer */
	while ( !(UCSRA_REG & (1<<UDRE_BIT))) /* check Data Register Empty bit */
		;
	UDR_REG = c; /* fill Data Register */
}
/***************************************************************************/
void serial_tx_string (char *s)
{
	char err = 0;

	while (err == 0 && *s != 0)
	{
		ob_head++;
		if (ob_head == OB_MAX)
			ob_head = 0;
		if (ob_head != ob_tail) /* do not add if buffer overrun */
			ob[ob_head] = *s;
		else
		{
			err = 1;
			ob_head = 0;
			ob_tail = 0;
		}
		s++; /* go to next char in s */
	}

	if (tx_busy == FALSE)
		serial_tx_init();
}
/***************************************************************************/
unsigned char serial_tx_idle (void)
{
	/* test if no transmission is in progress */
	return (UCSRA_REG & (1<<TXC_BIT)); /* check Transmit Complete bit */
}
/***************************************************************************/
unsigned char serial_rx_avail (void)
{
	/* return true if there is a character in the input buffer */
	return (ib_head != ib_tail); 
}
/***************************************************************************/
unsigned char serial_rx (void)
{
	/* return next char in buffer */
	ib_tail++;
	if (ib_tail == IB_MAX)
		ib_tail = 0;
	return (ib[ib_tail]);
}
/***************************************************************************/
void serial_rx_flush (void)
{
	unsigned char c;
	while (UCSRA_REG & (1<<RXC_BIT))
		c = UDR_REG;
}
/***************************************************************************/
