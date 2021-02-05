/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016 - 2018, NXP.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * Description:
 * ==================================================================================
 * The FlexIO is a highly configurable module providing a wide range of functionality.
 * This example configures two timers and one pin to emulate:
	• PWM signal at 100KHz, duty cycle as Init function parameter.
	• UART 8-bits Transfer at 19200 b/s (polling technique).
 */


#include "device_registers.h"
#include "FlexIO.h"
#include "clocks_and_modes.h"

void PORT_init (void)
{
	/*!
	 * Pins definitions
	 * ===================================================
	 *
	 * Pin number        | Function
	 * ----------------- |------------------
	 * PTD0				 | FXIO_D0
	 * PTD1              | FXIO_D1
	 * PTE15			 | FXIO_D2
	 */
  PCC->PCCn[PCC_PORTD_INDEX]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTD */
  PCC->PCCn[PCC_PORTE_INDEX]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTE */
  PORTD->PCR[0]  = PORT_PCR_MUX(6);	/* Port D1: MUX = FXIO_D0  */
  PORTD->PCR[1]  = PORT_PCR_MUX(6);	/* Port D1: MUX = FXIO_D1  */
  PORTE->PCR[15] = PORT_PCR_MUX(6);	/* Port E15: MUX = FXIO_D2 */
  PTD->PDDR |= 1<<0|1<<1;			/* Port D0, D1:  Data Direction = output */
  PTE->PDDR |= 1 << 15;				/* Port E15: Data Direction = output */
}

void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

int main(void)
{
	/*!
	 * Initialization
	 * ===================================================
	 */
	WDOG_disable();			/* Disable watchdog */
	SOSC_init_8MHz();		/* Initialize system oscilator for 8 MHz xtal */
	SPLL_init_160MHz();		/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz();	/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();			/* Configure ports */	
	FLEXIO_UART_init();		/*! Initialize Timer 0, Shifter 0 to emulate 
							 *	an UART transmitter instance, using:
							 *	FXIO_D2 to generate baudrate (output disabled).
							 *	FXIO_D1 as output shifter pin.
							 */

	FLEXIO_100KHz_PWM_init(50);	/*! Initialize Timer 1 and FXIO_D0
								 *	to emulate PWM. 
								 */

	/* Transmit example: send "Emulating UART by Using FlexIO module..." */
	FLEXIO_UART_transmit_string("Emulating UART by Using FlexIO module...");

	/*!
	 * Wait forever
	 * ========================
	 */
	for(;;)
	{

	}
}
