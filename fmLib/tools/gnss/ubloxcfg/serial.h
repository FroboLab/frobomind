/****************************************************************************
# Serial driver
# Copyright (c) 2004-2013, Kjeld Jensen, Cetus <kjeld@cetus.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name Cetus nor the
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
#****************************************************************************
# File: serial.h
# Purpose: Serial driver header file
# Project: Serial driver
# Platform: Linux/OSX
# Author: Kjeld Jensen <kjeld@cetus.dk>
# Note: You must have root priviledges to run these functions
# Created:  2004-07-06 Kjeld Jensen, 1.00
# Modified: 2004-08-09 Kjeld Jensen, 1.01, header changed
# Modified: 2005-02-05 Kjeld Jensen, 1.10, added serial_flush()
# Modified: 2008-04-17 Kjeld Jensen, 1.20, added ser_linux_baudrate, changed prefix to serb, moved info from platform.h to serial.h
# Modified: 2010-03-11 Kjeld Jensen, 1.30, Added support for baudrates 460800-4000000, removed TARGET defines as init has been modified
# Modified: 2011-02-06 Kjeld Jensen, 1.31, Released under MIT license
# Modified: 2011-03-28 Kjeld Jensen, 1.32, solved a minor problem at this header causing a compiler warning.
****************************************************************************/
/* system includes */
#include <stdlib.h>
#include <termios.h>

/***************************************************************************/
/* defines */

/* we need to define this if the GMUS-03 USB-serial adapter is used */
/* #define MACOSX_USBSERIAL */

/***************************************************************************/
/* function prototypes */

/* returns linux formattet baud rate */
long ser_linux_baudrate (long baudRate);

/* initializes the serial port, returns 0 if everything is ok */
int ser_open (
	int *serRef,			/* returned reference to the port */
	struct termios *oldtio,	/* placeholder for old port settings*/
	char *devName,			/* eg. "/dev/ttyS0" */
	long baudRate);

int ser_send (int serRef, void *buffer, int numBytes);

/* retrieves up to numBytes bytes from the serial port, returns the number
   of retrieved bytes, 0 if no bytes retrieved or -1 if an error occurs */
int ser_receive (int serRef, void *buffer, int numBytes);

/* flushes the input buffer */
void ser_flush (int serRef);

void ser_close (int serRef, struct termios oldtio);

/***************************************************************************/
