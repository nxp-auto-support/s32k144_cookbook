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
 * =================================================================
 * Initialization example, the LED will turn red if the input
 * voltage does not overcome the given threshold, and green if it does.
 */

#include "device_registers.h"
#include "clocks_and_modes.h"
#include "acmp.h"

#define PTD15 (15)
#define PTD16 (16)
void PORT_init (void)
{
	/*!
	 * Pins definitions
	 * ===================================================
	 *
	 * Pin number        | Function
	 * ----------------- |------------------
	 * PTE3              | [CMP0_OUT]
	 * PTA0				 | [CMP0_IN0]
	 * PTD15             | GPIO [RED LED]
	 * PTD16			 | GPIO [GREEN LED]
	 */

  PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT A  */
  PORTA->PCR[0] = PORT_PCR_MUX(0);	/* Port A0: default, Comparator Input Signal*/

	PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT D  */
  PORTD->PCR[PTD15] = PORT_PCR_MUX(1);  /* Port D15: MUX = GPIO */
  PORTD->PCR[PTD16] = PORT_PCR_MUX(1);  /* Port D16: MUX = GPIO */

  PTD->PDDR |= 1<<PTD15	    /* Port D15: Data Direction = output */
  	  	  	  |1<<PTD16;    /* Port D16: Data Direction = output */

  PTD->PSOR |= 1<<PTD15		/* Turn-Off RGB led */
		      |1<<PTD16;

  PCC->PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT D  */
  PORTE->PCR[3] = PORT_PCR_MUX(7);	/* Port E7: MUX = ALT7, Comparator Output trigger */
}


void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

void Enable_Interrupt(uint8_t vector_number)
{
  S32_NVIC->ISER[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
  S32_NVIC->ICPR[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

int main(void)
{
	/*!
	 * Initialization
	 * ===============================
	 */
  	WDOG_disable();		  	/* Disable WDOG */
  	PORT_init();            /* Configure ports */
  	SOSC_init_8MHz();       /* Initialize system oscilator for 8 MHz xtal */
  	SPLL_init_160MHz();     /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
  	NormalRUNmode_80MHz();  /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	ACMP_Init(127);					/* Initialize CMP0 */
	Enable_Interrupt(CMP0_IRQn);	/* Enable CMP0 interrupt */

	/*!
	 * Wait forever
	 * ========================
	 */
	for(;;)
	{


	}
}

void CMP0_IRQHandler (void)
{
	if(CMP0->C0 & CMP_C0_CFR_MASK)
	{
		/* A rising edge on COUT has occurred */
		CMP0->C0 |= CMP_C0_CFR_MASK;	/* CFR is cleared by writing 1 to it. */
		PTD->PSOR |= 1<<15;		/* Turn-Off RED led */
		PTD->PCOR |= 1<<16;		/* Turn-On GREEN led */
	}
	else if(CMP0->C0 & CMP_C0_CFF_MASK)
	{
		/* A falling edge on COUT has occurred */
		CMP0->C0 |= CMP_C0_CFF_MASK;	/* CFF is cleared by writing 1 to it. */
		PTD->PSOR |= 1<<16;		/* Turn-Off GREEN led */
		PTD->PCOR |= 1<<15;		/* Turn-On RED led */
	}

}



