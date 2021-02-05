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
 * =====================================================================
 * This short project is a starting point to learn GPIO.
 * An input is polled to detect a high or low level. An output is set
 * depending on input state. If running code on the S32K14x evaluation
 * board, pressing button 0 lights up the blue LED.
 */

#include "device_registers.h"

/*! Port PTD0, bit 0: FRDM EVB output to blue LED
 */
#define PTD0  0

/*! Port PTC12, bit 12: FRDM EVB input from BTN0 [SW2]
 */
#define PTC12 12

void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

int main(void)
{
	int counter = 0;

	/*!
	 * Pins definitions
	 * ===================================================
	 *
	 * Pin number        | Function
	 * ----------------- |------------------
	 * PTD0              | GPIO [BLUE LED]
	 * PTC12             | GPIO [SW2]
	 *
	 */


	/*!
	 * Initialization
	 * ===================================================
	 */

	WDOG_disable();/* Disable Watchdog in case it is not done in startup code */
	PCC-> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;/* Enable clocks to peripherals (PORT modules) */
	PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;/* Enable clock to PORT C*/
								/* Enable clock to PORT D*/

								/* Configure port C12 as GPIO input (BTN 0 [SW2] on EVB) */
	PTC->PDDR &= ~(1<<PTC12);   /* Port C12: Data Direction= input (default) */
	PORTC->PCR[12] = PORT_PCR_MUX(1)
					 |PORT_PCR_PFE_MASK; /* Port C12: MUX = GPIO, input filter enabled */

								/* Configure port D0 as GPIO output (LED on EVB) */
	PTD->PDDR |= 1<<PTD0;       /* Port D0: Data Direction= output */
	PORTD->PCR[0] = PORT_PCR_MUX(1); /* Port D0: MUX = GPIO */

	/*!
	 * Infinite for:
	 * ========================
	 */
		for(;;)
		{

			/*! -If Pad Data Input = 1 (BTN0 [SW2] pushed)
			 *
			 * Clear Output on port D0 (LED on)
			 *
			*/

			if (PTC->PDIR & (1<<PTC12)) {
				PTD-> PCOR |= 1<<PTD0;
			}
			else {							 /* -If BTN0 was not pushed*/
				PTD-> PSOR |= 1<<PTD0;       /*		Set Output on port D0 (LED off) */
			}
			counter++;
		}
}
