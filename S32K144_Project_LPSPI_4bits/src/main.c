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
 * =========================================================================================
 * LPSPI module supports 1/2/4/8 bit mode.
 *
 * This feature allows to establish a communication in a half-duplex mode with 4 data lines.
 *
 * In a real application, this capability is really useful in cases where you need to communicate
 * with a QSPI interface (like a memory or a display device) and the MCU doesn’t have QSPI module
 * or it is already used with another interface.
 *
 * In this example 4 data lines are used by the LPSPI at 1 MHz to show this capability.
 * The transfers are made via DMA.
 * */

#include "device_registers.h" 							/* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
#include "dma.h"
#include "LPSPI.h"


#define SIZE  4

/* Saving memory for 2 TCD structure (1 for LPSPI command transfer, 1 for data transfer) */
TCD_t TCDm[2] __attribute__ ((aligned (32)));

/* Dummy data for Blanking time */
#define Cmd        0x11101011
#define Addrs      0x07FF0000
#define DummyD0D1  0x0000AAAA
#define D2D3D4D5   0x5555AAAA

/* Data */
uint32_t Tx_Dn[SIZE] = { Cmd, Addrs, DummyD0D1, D2D3D4D5 };


/* Word of commands for SPI TCR register */
uint32_t lpspi_cmd[2] =
{
	  LPSPI_TCR_CPHA_MASK
	| LPSPI_TCR_PRESCALE(2)
	| LPSPI_TCR_PCS(1)
	| LPSPI_TCR_WIDTH(0x2)
	| LPSPI_TCR_FRAMESZ(31)
	| LPSPI_TCR_CONT_MASK,

	  LPSPI_TCR_CPHA_MASK
	| LPSPI_TCR_PRESCALE(2)
	| LPSPI_TCR_PCS(1)
	| LPSPI_TCR_WIDTH(0x2)
	| LPSPI_TCR_FRAMESZ(31),
 };


/*!
* @brief PORTn Initialization
*/
void PORT_init (void)
{
	/*!
	 *            Pins Definitions
	 * ===================================================
	 *
	 *    Pin Number     |    Function
	 * ----------------- |------------------
	 * PTD3              | LPSPI1 [PCS0]
	 * PTA6              | LPSPI1 [PCS1]
	 * PTB16             | LPSPI1 [SOUT]
	 * PTB14 			 | LPSPI1 [SCK]
	 * PTB15 			 | LPSPI1 [SIN]
	 * PTA16             | LPSPI1 [PCS2]
	 * PTB17             | LPSPI1 [PCS3]
	 */

	PCC -> PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT A */
	PCC -> PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT B */
	PCC -> PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT D */

	PORTD -> PCR[3] |= PORT_PCR_MUX(3); 				/* Port D3: MUX = ALT3, LPSPI1_PCS0 */
	PORTA -> PCR[6] |= PORT_PCR_MUX(3); 				/* Port A6: MUX = ALT3, LPSPI1_PCS1 */
	PORTB -> PCR[16] |= PORT_PCR_MUX(3); 				/* Port B16: MUX = ALT3, LPSPI1_SOUT */
	PORTB -> PCR[14] |= PORT_PCR_MUX(3); 				/* Port B14: MUX = ALT3, LPSPI1_SCK */
	PORTB -> PCR[15] |= PORT_PCR_MUX(3); 				/* Port B15: MUX = ALT3, LPSPI1_SIN */

	PORTA -> PCR[16] |= PORT_PCR_MUX(3);				/* Port A16: MUX = ALT3, LPSPI1_PCS2 */
	PORTB -> PCR[17] |= PORT_PCR_MUX(3); 				/* Port B17: MUX = ALT3, LPSPI1_PCS3 */
}

void WDOG_disable (void)
{
	WDOG -> CNT = 0xD928C520;     			/* Unlock watchdog */
	WDOG -> TOVAL = 0x0000FFFF;   			/* Maximum timeout value */
	WDOG -> CS = 0x00002100;    			/* Disable watchdog */
}

void Enable_Interrupt (uint8_t vector_number)
{
	S32_NVIC->ISER[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
	S32_NVIC->ICPR[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

int main (void)
{
	/*!
	 * Initialization:
	 */
	WDOG_disable();							/* Disable watchdog */
	SOSC_init_8MHz();      					/* Initialize system oscilator for 8 MHz xtal */
	SPLL_init_160MHz();    					/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); 					/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();							/* Initialize ports */

	/* LPSPI1 4 bit mode, FIFO enable and CS1 */
	LPSPI1_init_master();    				/* Initialize LPSPI 1 as master */
	LPSPI1_4bitMode_enable(); 				/* Enabled 4 bit mode and DMA requests */

	/* First we use the command with Continuous mode enable, length 32 and width 2 */
	LPSPI1 -> TCR = lpspi_cmd[0];


	/* eDMA Initialization */
	DMA_init();              				/* Initialize DMA controller */

	/* Save TCD for data transfer in index 0. Send Tx_Dn */
	DMA_TCDm_config ((uint32_t *)&Tx_Dn[0], (uint32_t *)&LPSPI1 -> TDR, SIZE, &(TCDm[0]));

	/* Scatter Gather enabled, when finished, it loads TCD index 1 */
	TCDm[0].CSR      &=~ (DMA_TCD_CSR_DREQ(1));
	TCDm[0].CSR      |=  DMA_TCD_CSR_ESG(1);
	TCDm[0].DLASTSGA =   DMA_TCD_DLASTSGA_DLASTSGA (&TCDm[1]);

	/* Save TCD for command change. CONT = 0 and the continuous transfer command to TCR is finished */
	DMA_TCDm_config ((uint32_t *)&lpspi_cmd[1], (uint32_t *)&LPSPI1 -> TCR, 1, &(TCDm[1]));

	/* Push TCD with index 0 to DMA channel 0 */
	DMA_TCD_Push(0, &TCDm[0]);

	/* Enable DMA Ch0 request */
	DMA -> SERQ = DMA_SERQ_SERQ(0);


	/*!
	* Infinite for:
	*/
	for(;;)
	{
	}
}
