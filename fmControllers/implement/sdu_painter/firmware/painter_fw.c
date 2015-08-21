/****************************************************************************
# Painter firmware - FroboMind Controller interface
# 
# Copyright (c) 2014-2015, Mikkel K. Larsen
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
# File: painter_fw.c
# Project: Painter firmware
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Author: Mikkel K. Larsen
# Created:  2014-10-23 Mikkel K. Larsen
# Modified: 
******************************************************************************/
/* includes */
#include "painter_fw.h"

/***************************************************************************/


void painter_fw_Init(void)
{
	   //Configure TIMER1
	   TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	   TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)

	   ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	   //WM_SPRAY_PIN_INIT;
}

void painter_fw_Update(int spray_signal)
{
	PWM_SPRAY_PIN_INIT;
	if (spray_signal == 0){
		spray_state = SPRAY_POWER_LESS;
	}
	else if (spray_signal == 1){
		spray_state = SPRAY_ON;
		}
	else if (spray_signal == 2){
		spray_state = SPRAY_OFF;
		}
	else {
		spray_state = SPRAY_POWER_LESS;
		}

	switch (spray_state)
	{
		case SPRAY_POWER_LESS:
			OCR1A=PWM_SPRAY_OFF;   //PWM 2 ms
			break;
			
		case SPRAY_ON:
			OCR1A=PWM_SPRAY_ON;   //PWM 1.3mS
			break;

		case SPRAY_OFF:
			OCR1A=PWM_SPRAY_OFF;   //PWM 2 ms
			break;

		default:
			OCR1A=PWM_SPRAY_OFF;   //PWM 2 ms
			break;

	}



}

