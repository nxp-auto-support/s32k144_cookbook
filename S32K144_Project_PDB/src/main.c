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
 * ======================================================================
 * The ADC is initialized for HW triggers, and the PDB will trigger it
 * with a small delay that can be noticed by the toggling LED's
 * 
 * 
 */


#include "device_registers.h" /* include peripheral declarations */
#include "pdb.h"
#include "ADC.h"
#include "clocks_and_modes.h"

#define PTD15 15 /* RED LED*/
#define PTD16 16 /* GREEN LED*/
#define PTD0 0   /* BLUE LED */


void PORT_init (void) {
	PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORT D */

	PORTD->PCR[PTD0]  =  0x00000100;  /* Port D0: MUX = GPIO */
	PORTD->PCR[PTD15] =  0x00000100;  /* Port D15: MUX = GPIO */
	PORTD->PCR[PTD16] =  0x00000100;  /* Port D16: MUX = GPIO */

	PTD->PDDR |= 1<<PTD0;       	  /* Port D0:  Data Direction= output */
	PTD->PDDR |= 1<<PTD15;          /* Port D15: Data Direction= output */
	PTD->PDDR |= 1<<PTD16;          /* Port D16: Data Direction= output */
}
void WDOG_disable (void){
  WDOG->CNT=0xD928C520;     /* Unlock watchdog */
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value */
  WDOG->CS = 0x00002100;    /* Disable watchdog */
}


int main(void)
{
	WDOG_disable();			/*Disable Watchdog*/
	S32_NVIC->ISER[1] = 1 << (ADC0_IRQn % 32);  /* IRQ48-LPIT0 ch0: enable IRQ */
	SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
	SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
	PORT_init();
	ADC_init_HWTrigger(12);
	PDB0_init();

	for(;;)
	{
	}

}


void ADC0_IRQHandler(void)
{
	volatile uint16_t ADC_result = 0;

	if(ADC0->SC1[0] & ADC_SC1_COCO_MASK)
	{
		ADC_result=ADC0->R[0];      /* For SW trigger mode, R[0] is used */
		 if (ADC_result > 3072) {           /* If result > 3.75V */
		      PTD->PSOR |= 1<<PTD0 | 1<<PTD16;    /* turn off blue, green LEDs */
		      PTD->PTOR |= 1<<PTD15;              /* turn on red LED */
		    }
		    else if (ADC_result > 2048) {      /* If result > 3.75V */
		      PTD->PSOR |= 1<<PTD0 | 1<<PTD15;    /* turn off blue, red LEDs */
		      PTD->PTOR |= 1<<PTD16;     	      /* turn on green LED */
		    }
		    else if (ADC_result >1024) {       /* If result > 3.75V */
		      PTD->PSOR |= 1<<PTD15 | 1<<PTD16;   /* turn off red, green LEDs */
		      PTD->PTOR |= 1<<PTD0;     	      /* turn on blue LED */
		    }
		    else {
		      PTD->PSOR |= 1<<PTD0 | 1<< PTD15 | 1<<PTD16; /* Turn off all LEDs */
		    }
	}
}
