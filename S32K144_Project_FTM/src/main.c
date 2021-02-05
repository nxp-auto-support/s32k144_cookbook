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
 * ============================================================================================
 * This example uses the Flex Timer Module (FTM) to perform common digital I/O functions of:
	• Edge-Aligned Pulse Width Modulation (EPWM): 1 Hz, low 25%, high 75%
	• Output Compare (OC): Toggle output every 100 msec (10 MHz toggle produces 5 MHz frequency)
	• Input Capture (IC): Capture input’s rising or falling edge times
 * 
 * All channels in the FTM share a common 16-bit counter for the I/O functions.
 * */

#include "device_registers.h"          
#include "clocks_and_modes.h"
#include "FTM.h"

void PORT_init (void)
{
	/*!
	 * Pins definitions
	 * ===================================================
	 *
	 * Pin number        | Function
	 * ----------------- |------------------
	 * PTE8              | FTM0CH6
	 * PTD15             | FTM0CH0
	 * PTD16			 | FTM0CH1
	 */
  PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTD */
  PCC->PCCn[PCC_PORTE_INDEX ]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTE */
  PORTE->PCR[8]|=PORT_PCR_MUX(2);           		/* Port E8: MUX = ALT2, FTM0CH6 */
  PORTD->PCR[15]|=PORT_PCR_MUX(2);           		/* Port D15: MUX = ALT2, FTM0CH0 */
  PORTD->PCR[16]|=PORT_PCR_MUX(2);           		/* Port D16: MUX = ALT2, FTM0CH1 */
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
	 * Initialization:
	 * =======================
	 */
  WDOG_disable();        /* Disable WDOG*/
  SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz SPLL & core, 40 MHz bus, 20 MHz flash */
  
  FTM0_init();           /* Init FTM0 using 8 MHz SOSCDIV1_CLK */
  FTM0_CH0_OC_init();    /* Init FTM0 CH0, PTD15 */
  FTM0_CH1_PWM_init();   /* Init FTM0 CH1, PTD16 */
  FTM0_CH6_IC_init();    /* Init FTM0 CH6, PTE8  */
  PORT_init();           /* Configure ports */
  FTM0_start_counter();  /* Start FTM0 counter */

	/*!
	 * Infinite for:
	 * ========================
	 */
	  for(;;)
	  {
		FTM0_CH0_output_compare();/* If output compare match: */
								  /* Pin toggles (automatically by hardware) */
								  /* Clear flag 8 */
								  /* Reload timer */
		FTM0_CH6_input_capture(); /* If input captured: clear flag, read timer */
	  }
}
