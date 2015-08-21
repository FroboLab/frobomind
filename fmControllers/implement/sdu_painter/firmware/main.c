/****************************************************************************
# painter_fw FroboMind Controller interface
# Control unit FroboLightHouse.
# Copyright (c) 2014-2016, Mikkel K. Larsen
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
# File: main.c
# Project: painter_fw_SW
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Fusebits: EXTENDED: 0xFD , HIGH: 0xC9 , LOW: 0xFF
# Author: Mikkel K. Larsen
# Created:  2015-04-10 Mikkel K. Larsen
# Modified: 
*****************************************************************************
# Most of this file is a modifikation of the file created by Kjeld Jensen.
# The source is the project: Frobit FroboMind Controller interface 
# Author: Kjeld Jensen <kjeld@frobomind.org>
# Created:  2014-08-28 Kjeld Jensen
****************************************************************************/
/* includes */
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "fmctrl_def.h"
#include "avr_serial.h"
#include "rcnmea.h"
#include "painter_fw.h"

/***************************************************************************/
/* defines */

#define false					0
#define true					1

/* defines for the timer/counter0 interrupt */
#define INT0_CNT_TOP			249 /* corresponds on an interrupt0 each 1ms */
#define FLIPBIT					PE1
#define FLIPBIT_PORT			PORTE
#define FLIPBIT_DDR				DDRE

/* system state (only the highest status is transmitted through NMEA) */
#define STATE_OK				1
#define STATE_WARN_NMEA_CS		2 /* error occured while receiving the latest nmea packet */
#define STATE_ERR_NO_CONFIG		3 /* no configuration received yet */
#define STATE_ERR_WATCHDOG		4 /* no validated nmea packets received for the past 0.2 seconds */
#define STATE_ERR_LOWBAT		5 /* battery voltage too low */
#define STATE_ERR_ACTUATOR		6 /* battery voltage too low */

#define VOLTAGE_MIN_DEFAULT		320 /* default minimum voltage (12V) */

/* signal led defines */
#define LED_STATE_OFF			0
#define LED_STATE_ON			1
#define LED_DELAY				4 /* times the cycle */

/* adc defines */
#define ADC_NUM					1 /* number of used ADC's */
#define ADC_VOLT				0 /* voltage measurement */
#define ADC_IN1					1 /* analog in 1 */
#define ADC_IN2					2 /* analog in 2 */

/* adc port defines */
#define ADC_PORT_IN1 			1
#define ADC_PORT_IN2			2

/* NMEA defines */
#define NMEA_WD_TOUT			2 /* 1/10 [s] without receiving ok NMEA before timeout */


/***************************************************************************/
/* global and static variables */

/* timer1 and scheduler variables */
volatile unsigned char t1ms;
unsigned short t1ms_cnt;

/* system variables */
static unsigned char reset_source; /* 0=pow,1=ext,2=brownout,3=wd,4=jtag */
char state;

/* user interface variables*/
char led_state;
char led_signal;
char led_count;
char but1;

/* NMEA variables */
char painter_fw_param_received;
unsigned short nmea_wd_timeout; /* NMEA watchdog timeout [ms] */
unsigned short nmea_wd; /* NMEA watchdog counter */
unsigned short pflst_interval; /* PFLST interval (20-10000) [ms] */
long nmea_ticks_l, nmea_ticks_r;

/* ADC variables (10 bit [0;1023]) */
volatile unsigned short adc_data[ADC_NUM]; /* ADC data variables */
volatile unsigned char adc_ch; /* current adc channel */
unsigned char adc_ports[ADC_NUM]; /* maps to ADC ports */

unsigned short voltage;
unsigned short voltage_min;
unsigned char battery_low_warning;

/* global and static variables */
extern char state;

/***************************************************************************/
void sched_init(void)
{
	/* timer 0 interrupt init (each 1ms) */
	t1ms = 0;
	t1ms_cnt = 0;
    TIMSK0 = BV(OCIE0A); 
    TCCR0A = BV(CS00) | BV(CS01) | BV(WGM01); /* clk/64, TOS is defined as OCR0A */  
    OCR0A = INT0_CNT_TOP;
	/* PB_OUT (FLIPBIT_DDR, FLIPBIT); */ /* set 1ms flipbit as output */
}
/***************************************************************************/
/*ISR(SIG_OUTPUT_COMPARE0A) */
ISR (TIMER0_COMP_vect)
{
	t1ms++;
	/* PB_FLIP (FLIPBIT_PORT, FLIPBIT); */ /* time to flip the flip bit */
}
/***************************************************************************/
/* ADC interrupt handler */
ISR (ADC_vect)
{
	adc_data[adc_ch] = ((ADCL) | ((ADCH)<<8)); /* read value */
	if (++adc_ch >= ADC_NUM) /* go to next adc channel */
		adc_ch = 0;
	ADMUX = (1<<REFS0) | adc_ports[adc_ch];
	ADCSRA |= (1<<ADSC);  /* request a new adc conversion */
}
/***************************************************************************/
void adc_init (void)
{
	adc_ch = 0;
	adc_ports[0] = ADC_PORT_VOLT; /* map voltage measurement to ADC0 */
	adc_ports[1] = ADC_PORT_IN1; /* map analog in 1 to ADC1 */
	adc_ports[2] = ADC_PORT_IN2; /* map analog in 2 to ADC2 */

	ADCSRA = BV(ADEN); /* enable ADC conversion */
	ADCSRA |= (BV(ADPS2) | BV(ADPS1) | BV(ADPS0)); /* div by 128 presc. */
	ADCSRA |= BV(ADIE); /* interrupt enable */
	ADMUX = BV(REFS0)| adc_ports[adc_ch]; /* Voltage reference is AREF) */
	ADCSRA |= BV(ADSC); /* request ADC conversion */
}
/***************************************************************************/
void voltage_update(void)
{
		if (battery_low_warning == false && voltage < voltage_min)
			battery_low_warning = true;
		else if (battery_low_warning == true && voltage >= voltage_min)
			battery_low_warning = false;
}
/***************************************************************************/
void led_update(void)
{
	/* led_state = state; */
	switch (led_state) {
		case LED_STATE_ON:
			led_state = LED_STATE_OFF;
			INT_LED_OFF;
			break;

		case LED_STATE_OFF:
			led_count++;
			if (led_count <= led_signal) {
				INT_LED_ON;
				led_state = LED_STATE_ON;
			}
			else if (led_count > led_signal + LED_DELAY) {
				led_count = 0;
			}
			break;
	}
}

/***************************************************************************/
void led_init(void)
{
	INT_LED_INIT;
	led_count = 0;
	led_signal = 1;

	led_state = LED_STATE_OFF;
}
/***************************************************************************/
void button_update(void)
{
	but1 = PB_IS_HIGH (PINA, PINA6); /* button enabled if logic zero */
	but1 = PB_IS_HIGH (PINA, PINA7); /* button enabled if logic zero */
}
/***************************************************************************/
void button_init(void)
{
	PB_PULL_UP (PORTA, PA6); /* enable pull-up resistor */
	PB_PULL_UP (PORTA, PA7); /* enable pull-up resistor */
	button_update();
}
/***************************************************************************/
void nmea_init(void)
{
	nmea_reset();
	nmea_wd = 0xffff; /* set watchdog timeout at init */

	tx[0] = '$'; /* send first boot message */
	tx[1] = 'P';
	tx[2] = 'F';
	tx[3] = 'R';
	tx[4] = 'H';
	tx[5] = 'I';
	tx[6] = ',';
	tx[7] = '2'; /* hw version */		
	tx[8] = ',';
	tx[9] = '1'; /* sw major version */	
	tx[10] = ',';
	tx[11] = '1'; /* sw minor version */
	tx[12] = ',';
	tx[13] = '0' + reset_source; /* latest reset type */
	tx_len = 14;
	nmea_tx();

	tx[4] = 'S'; /* prepare for status messages */
	tx[5] = 'T';
	tx_len = 7;

	nmea_ticks_l = 0;
	nmea_ticks_r = 0;
}
/***************************************************************************/
void nmea_rx_parse(void)
{		/* Commonication for painter_fw*/
	if (rx[0] == 'P' && rx[1] == 'F' && rx[2] == 'R') 		
	{
		if (rx[3] == 'S' && rx[4] == 'P') /* System Parameters */
		{
			rx_ite = 5; /* jump to first value */
			voltage_min = nmea_rx_next_val();
		}
		else if (rx[3] == 'C' && rx[4] == 'P') /* Communication Parameters */
		{
			rx_ite = 5; /* jump to first value */
			pflst_interval = nmea_rx_next_val();
			if (rx_ite != -1)
				nmea_wd_timeout = nmea_rx_next_val();
		}
		else if (rx[3] == 'R' && rx[4] == 'S') /* Relay Settings */
		{
			rx_ite = 5; /* jump to first value */
			nmea_wd = 0; /* reset watchdog timeout */
			spray_signal = ((nmea_rx_next_val()));		//Set Spray_Signal [0-2]
			painter_fw_param_received = true;

		}

	}
}
/***************************************************************************/
void nmea_tx_status(void)
{
//	long tl, tr;
	tx_len = 7; /* keep the NMEA message prefix */

	nmea_tx_append_ushort (state);

	nmea_tx_append_ushort (voltage); /* battery voltage [0;1023] */

	nmea_tx_append_ushort (spray_signal); /*  [0;1023] */

	tx_len--; /* delete the last comma */
	nmea_tx();
}
/***************************************************************************/
void state_update(void)
{
	if (battery_low_warning == true)
	{
		state = STATE_ERR_LOWBAT;
	}
	else if (nmea_wd > NMEA_WD_TOUT)
	{
		// Blokker, gemt til vedligehold, saettes foran "state = STATE_ERR_WATCHDOG;" : //test-melder ikke fejl paa wathdog -
		
		state = STATE_ERR_WATCHDOG;
		painter_fw_param_received = false;
		spray_signal = 0;
	}
	else if (painter_fw_param_received == false)
	{
		state = STATE_ERR_NO_CONFIG; 
		spray_signal = 0;
	}
	else if (nmea_err != 0)
	{
		state = STATE_WARN_NMEA_CS;
		nmea_err = 0;
		spray_signal = 0;
	}		
	else
	{
		state = STATE_OK;
	}

	led_signal = state; /* Frobomind Controller LED flashes state number */
}
/***************************************************************************/
void sched_update (void)
{
	t1ms_cnt++;
	if (t1ms_cnt == 10000)
		t1ms_cnt = 0;
	

	/* each 10 ms */
	if (t1ms_cnt % 10 == 0) /* each 10 ms */
	{
		wdt_reset(); /* reset watchdog */

		if (t1ms_cnt % 20 == 0) /* each 20 ms */
		{

		}

		if (t1ms_cnt % 50 == 0) /* each 50 ms */
		{
		}

		if (t1ms_cnt % 100 == 0) /* each 100 ms */
		{
			if (nmea_wd_timeout)
				nmea_wd++; /* increase nmea watchdog timeout */
			else
				nmea_wd = 0;
			voltage = adc_data[0]; /* read voltage measurement */
			state_update();
		}
		if (t1ms_cnt % 200 == 0) /* each 200 ms */
		{
			button_update();		
			led_update();
			painter_fw_Update(spray_signal);
			voltage_update();
			nmea_tx_status();
		}
	}
}

/***************************************************************************/
void save_reset_source(void)
{
	char reset_reg = MCUSR; /* save the source of the latest reset */
	MCUSR = 0;
	switch (reset_reg) 
	{
		case 1: /* power on */
			reset_source = 0; break;
		case 2: /* reset activated */
			reset_source = 1; break;
		case 4: /* brown out */
			reset_source = 2; break;
		case 8: /* watchdog */
			reset_source = 3; break;
		case 16: /* jtag */
			reset_source = 4; break;
	}
}
/***************************************************************************/
int main(void)
{
	save_reset_source(); /* determine the cause of the startup */
	sched_init(); /* initialize the scheduler */
	led_init(); /* initialize led */
	button_init(); /* initialize button */
	adc_init(); /* initialize ADC (battery voltage measurement) */
	serial_init(); /* initialize serial communication */
	painter_fw_Init(); /* initialize painter_fw */
	pflst_interval = 20; /* send $PFLST at 20 ms interval */
	nmea_wd_timeout = 1; /* set PFLCT watchdog timeout to 100ms */
	nmea_wd = NMEA_WD_TOUT+1; /* make sure we begin in watchdog timeout state */
	voltage_min = VOLTAGE_MIN_DEFAULT;
	battery_low_warning = false;
	state_update();
	sei(); /* enable interrupts */
	wdt_enable (WDTO_15MS); /* enable watchdog reset at approx 15 ms (ref. p.58) */
	nmea_init(); /* initialize nmea protocol handler */

	for (;;) /* go into an endless loop */
	{
		/* motor_update(); */

		if (t1ms != 0) /* if the interrupt has timed out after 10ms */
		{
			t1ms --;
			sched_update(); /* run the scheduler */
		}
		else
		{
			nmea_rx_update();
		}
	}
	return 0; /* just for the principle as we never get here */
}
/***************************************************************************/
