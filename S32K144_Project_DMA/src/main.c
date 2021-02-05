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
 * =============================================================================================
 * Initialize an eDMA channel’s Transfer Control Descriptor (TCD) to transfer a string of bytes
 * (“Hello world”) from an array in SRAM to a single SRAM byte location. This emulates a common 
 * use of DMA, where a string of data or commands is transferred automatically under DMA control 
 * to an input register of a peripheral. The intent of this example is to illustrate how to set 
 * up a DMA transfer.
 * */

#include "device_registers.h"  /* Include peripheral declarations */
#include "dma.h"
#include "clocks_and_modes.h"

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
  WDOG_disable();        /* Disable WDOG */
  SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

  DMA_init();              /* Init DMA controller */
  DMA_TCD_init();          /* Init DMA Transfer Control Descriptor(s) */

  DMA->SSRT = 0;           /* Set chan 0 START bit to initiate first minor loop */
  while (((DMA->TCD[0].CSR >> DMA_TCD_CSR_START_SHIFT) & 1) |    	/* Wait for START = 0 */
         ((DMA->TCD[0].CSR >> DMA_TCD_CSR_ACTIVE_SHIFT)  & 1))  {} 	/* and ACTIVE = 0 */
                                     	 	 	 	 	 	 	 	/* Now minor loop has completed */

  while (!((DMA->TCD[0].CSR >> DMA_TCD_CSR_DONE_SHIFT) & 1) ) {    /* Loop till DONE = 1 */
    /* Place breakpoint at next instruction & observe expressions TCD0_Source, TCD0_Dest */
    DMA->SSRT = 0;	/* Set chan 0 START bit to initiate next minor loop */

    while (((DMA->TCD[0].CSR >> DMA_TCD_CSR_START_SHIFT) & 1) |    		/* Wait for START = 0 */
           ((DMA->TCD[0].CSR >> DMA_TCD_CSR_ACTIVE_SHIFT)  & 1))  {} 	/* and ACTIVE = 0 */
                                     	 	 	 	 	 	 	 	 	/* Now minor loop has completed */
  }

  DMA->TCD[0].CSR &= ~(DMA_TCD_CSR_DONE_MASK);   /* Clear DONE bit */

	/*!
	 * Wait forever
	 * =======================
	 */
  while (1) {}
}
