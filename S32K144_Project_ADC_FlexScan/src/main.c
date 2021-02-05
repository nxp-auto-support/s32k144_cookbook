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
 * ======================================================================================================================
 * This example intends to show how to combine ADC, DMA and PDB to implement an ADC flexible storage.
 * In this project, PDB triggers ADC0 CH0 measurements which will be saved inside an internal memory buffer through DMA,
 * this way the MCU doesn't need to read the ADC result register because the transfers will be done by DMA.
 * The ADC readings are stored in the ADC_Results[] array inside the dma.c driver.
 * */

#include "device_registers.h"
#include "dma.h"
#include "clocks_and_modes.h"
#include "pdb.h"
#include "ADC.h"

void WDOG_disable (void)
{
	WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
	WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
	WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

int main(void)
{
	WDOG_disable();        			/* Disable WDOG */
	SOSC_init_8MHz();      			/* Initialize system oscillator for 8 MHz xtal */
	SPLL_init_160MHz();    			/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz();			/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
	ADC_FlexScan_Config();			/* Initialize ADC0 CH0 with HW Trigger and DMA Request */
	DMAMUX_FlexScan_init();			/* Initialize DMA to take requests from ADC0	*/
	DMA_TCD_FlexScan_Config();		/* Set up TCD CH0 to save measurements from ADC0 and link to CH1 to change ADC0 channel to measure */
	DMA->SERQ = DMA_SERQ_SERQ(0);	/* Enable Requests for DMA Channel 0 */

	PDB_FlexScan_Config();			/* Configure PDB to trigger ADC0 every second */

	S32_NVIC->ISER[0/32] |= 1<<(0%32);	/*	Enable interruption for DMA CH0	*/

        for(;;) {       

        }

	return 0;
}

void DMA0_IRQHandler (void) {
	DMA->CDNE = DMA_CDNE_CDNE(0);	/* Clear Done Status Flag of DMA Channel 0 */
	DMA->CINT = DMA_CINT_CINT(0);	/* Clear Interruption request flag of DMA Channel 0 */
	PDB0->SC &=~ PDB_SC_PDBEN_MASK;	/* Turn off PDB to stop getting samples after DMA CH0 major loop */
}
