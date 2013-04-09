/*****************************************************************************
# Ublox configuration loader
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
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
#****************************************************************************/

/* includes */
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "serial.h"

/* defines */
#define BUFSIZE 				20000
#define LINESIZE 				500
#define TX_RETRY 				20
#define RESPONSE_TIMEOUT		50 /* ms */
#define BREAK_AFTER_TIMEOUT		0 /* ms */

/* ubx protocol */
#define UBX_HEADER1				0xb5
#define UBX_HEADER2				0x62
#define UBX_ACK_ID1				0x05
#define UBX_ACK_ID2				0x01
#define UBX_NACK_ID2			0x00

/* response results */
#define ACK_OK					0
#define ACK_TIMEOUT				1
#define ACK_REJECT				2
#define ACK_STRANGE				3

/* global variables */
static int ser_ref;
static unsigned char tx[BUFSIZE];
static unsigned char rx[BUFSIZE];
static int errors = 0;

/***************************************************************************/
unsigned long mseconds ()
{
	struct timeval tv;

	gettimeofday (&tv, NULL);
	return ((tv.tv_sec % 86400) * 1000 + tv.tv_usec/1000);
}
/***************************************************************************/
void wait_mseconds (unsigned long wait)
{
	unsigned long timeout = mseconds() + wait;
	while (mseconds() <= timeout)
		;
}
/***************************************************************************/
int ubx_check_response()
{
	unsigned long start = mseconds();
	int status = ACK_TIMEOUT;
	int num_bytes;
	int rx_len = 0;

	while (start + RESPONSE_TIMEOUT >= mseconds() && status == ACK_TIMEOUT)
	{
		num_bytes = BUFSIZE - rx_len; /* check for new characters */
		num_bytes = ser_receive (ser_ref, rx + rx_len, num_bytes);
		if (num_bytes > 0) /* if we received new characters */
		{
			int i;
			rx_len += num_bytes;
			for (i=3; i<rx_len; i++) /* search for ACK or NACK packet */
			{
				if (rx[i-3] == UBX_HEADER1 && rx[i-2] == UBX_HEADER2 && rx[i-1] == UBX_ACK_ID1)
				{ /* return result */
					switch (rx[i])
					{
						case UBX_ACK_ID2:
							status = ACK_OK;
							break;
						case UBX_NACK_ID2:
							status = ACK_REJECT;
							break;
						default:
							status = ACK_STRANGE;
					}
				}
			}
		}
	}
	return status;
}
/***************************************************************************/
void add_checksum(int length)
{
	unsigned char cs_a = 0;
	unsigned char cs_b = 0;
	int i;
	for(i=2; i<length; i++)
	{
	    cs_a += tx[i];
    	cs_b += cs_a;
	}
	tx[length++] = cs_a;
	tx[length++] = cs_b;	
}
/***************************************************************************/
unsigned char ch_to_hex(unsigned char ch)
{
	if (ch >= '0' && ch <= '9')
		ch -= '0';
	else
	{
		 if (ch >= 'A' && ch <= 'F')
			ch -= 'A';
		else /* no range (error) check implemented */
			ch -= 'a';
		ch += 10;
	}
	return ch;
}
/****************************************************************************/
int main (int argc, char **argv)
{
	struct termios oldtio;
	int ser_result;

		printf ("\nUbloxCFG v2013-04-16\n");
		printf ("Copyright 2013 Kjeld Jensen <kjeld@frobomind.org>\n");
	if (argc != 4)
		printf ("\nUsage: ubloxcfg device baudrate file\n\n");
	else
	{
		unsigned long baudrate;
		printf ("\nDevice:   %s\n", argv[1]);
		baudrate = atoi (argv[2]);
		printf ("Baudrate: %ld\n", baudrate);
		printf ("File:     %s\n\n", argv[3]);

		ser_result = ser_open (&ser_ref, &oldtio, argv[1], ser_linux_baudrate(baudrate));
		if (ser_result == 0)
		{
			FILE *f;
			printf ("Serial device opened\n");
			tx[0] = UBX_HEADER1;
			tx[1] = UBX_HEADER2;

			f = fopen (argv[3], "r");
			if (f != NULL)
			{
				char line[LINESIZE];
				printf ("File opened\n");

				while (fgets ( line, sizeof line, f ) != NULL ) /* for all lines in the file */
				{
					int len = strlen(line);

					if (len > 11 && line[0] == 'C' && line[1] == 'F' && line[2] == 'G') /* only lines beginning with CFG- are relevant */
					{
						int i = 4;
						while (line[i] != '-' && i<len-3) /* search for the dash between name and numbers */
							i++;
						if (line[i] == '-')
						{
							int j = 2;
							line[i+1] = 0; /* hack: terminate the string to print out the text part */
							printf ("Sending: %s ", line);
							while (i+3 < len) /* store and print all hex values */
							{
								i += 3;
								tx[j++] = ch_to_hex(line[i-1])*16 + ch_to_hex(line[i]);
								printf ("%02x ", tx[j-1]);
							}
							add_checksum(j);
							/* printf ("Checksum %02x %02x ", tx[j], tx[j+1]); */
							j+= 2;
							tx[j] = 0;

							{
								int try = 0;
								int result = ACK_TIMEOUT;
							
								while (try < TX_RETRY && result != ACK_OK && result != ACK_REJECT && result != ACK_REJECT)
								{
									try++;
									ser_flush (ser_ref);
									ser_send (ser_ref, tx, j); /* send data to ublox */
									result = ubx_check_response(); /* check for ACK-ACK or ACK-NAK response */
									switch (result)
									{
										case ACK_OK:
											printf ("OK ");
											break;						
										case ACK_TIMEOUT:
											printf ("TIMEOUT ");
											 wait_mseconds (BREAK_AFTER_TIMEOUT);
											break;						
										case ACK_REJECT:
											printf ("REJECT ");
											break;						
										case ACK_STRANGE:
											printf ("STRANGE ");
											break;						
									}
								}
								if (result != ACK_OK && result != ACK_REJECT) /* if timeout or strange reponse we report the error */
									errors++;
							}
							printf ("\n");
						}
					}
				}
				printf ("Errors: %d\n", errors);
				fclose (f);
				printf ("File closed\n");
			}
			else
				printf ("Unable to open file!\n");

			ser_close (ser_ref, oldtio);
			printf ("Serial device closed\n");
		}
		else
			printf ("Unable to open serial device!\n");
		printf ("Quit\n\n");
	}	
	return (errors);
}
/****************************************************************************/

