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
# Modifieed 2011-03-14 Kjeld Jensen, added serial_rx_flush()
# Modified  2011-12-18 Kjeld Jensen, added serial_tx_idle
# Modified  2013-01-15 Kjeld Jensen, added support for double speed baud rates
# Modified  2013-02-04 Kjeld Jensen, migrated to the BSD license
# Modified  2014-02-21 Kjeld Jensen, added serial_tx_direct()
#****************************************************************************/

#ifndef _SERIAL_H
#define _SERIAL_H

/****************************************************************************/
/* function prototypes */

void serial_init(void);
void serial_tx (unsigned char c);
void serial_tx_string (char *s);
void serial_tx_direct (unsigned char c);
unsigned char serial_tx_idle (void);
unsigned char serial_rx_avail (void);
unsigned char serial_rx (void);
void serial_rx_flush (void);

/****************************************************************************/
#endif
