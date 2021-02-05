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

#include "device_registers.h"	  /* include peripheral declarations */
#include "LPSPI.h"
#include "STMPE610.h"
#include "TFT_LCD.h"

void LPSPI1_init_master(void)
{
	/*!
	 * LPSPI1 Clocking:
	 * ===================================================
	 */
	PCC->PCCn[PCC_LPSPI1_INDEX] = 0;          		/* Disable clocks to modify PCS ( default) 	*/
	PCC->PCCn[PCC_LPSPI1_INDEX] = PCC_PCCn_PR_MASK	/* (default) Peripheral is present.			*/
								 |PCC_PCCn_CGC_MASK	/* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) 	*/
								 |PCC_PCCn_PCS(6);
	/*!
	 * LPSPI1 Initialization:
	 * ===================================================
	 */
  LPSPI1->CR    = 0x00000000;   			/* Disable module for configuration 			*/
  LPSPI1->IER   = 0x00000000;   			/* Interrupts not used 						*/
  LPSPI1->DER   = 0x00000000;   			/* DMA not used 								*/
  LPSPI1->CFGR0 = 0x00000000;   			/* Defaults: 									*/
                                			/* RDM0=0: rec'd data to FIFO as normal 		*/
                                			/* CIRFIFO=0; Circular FIFO is disabled 		*/
                                			/* HRSEL, HRPOL, HREN=0: Host request disabled */

  LPSPI1->CFGR1 = LPSPI_CFGR1_MASTER_MASK;  /* Configurations: master mode									*/
											/* PCSCFG=0: PCS[3:2] are enabled 								*/
											/* OUTCFG=0: Output data retains last value when CS negated	*/
											/* PINCFG=0: SIN is input, SOUT is output 						*/
											/* MATCFG=0: Match disabled 									*/
											/* PCSPOL=0: PCS is active low 								*/
											/* NOSTALL=0: Stall if Tx FIFO empty or Rx FIFO full 			*/
											/* AUTOPCS=0: does not apply for master mode 					*/
											/* SAMPLE=0: input data sampled on SCK edge 					*/
											/* MASTER=1: Master mode 										*/

  LPSPI1->TCR   = LPSPI_TCR_CPHA_MASK
				  |LPSPI_TCR_PRESCALE(2)
				  |LPSPI_TCR_PCS(3)
				  |LPSPI_TCR_FRAMESZ(15);   /* Transmit cmd: PCS3, 16 bits, prescale func'l clk by 4, etc	*/
											/* CPOL=0: SCK inactive state is low 							*/
											/* CPHA=1: Change data on SCK lead'g, capture on trail'g edge	*/
											/* PRESCALE=2: Functional clock divided by 2**2 = 4 			*/
											/* PCS=3: Transfer using PCS3 									*/
											/* LSBF=0: Data is transfered MSB first 						*/
											/* BYSW=0: Byte swap disabled 									*/
											/* CONT, CONTC=0: Continuous transfer disabled 				*/
											/* RXMSK=0: Normal transfer: rx data stored in rx FIFO 		*/
											/* TXMSK=0: Normal transfer: data loaded from tx FIFO 			*/
											/* WIDTH=0: Single bit transfer 								*/
											/* FRAMESZ=15: # bits in frame = 15+1=16 						*/

  LPSPI1->CCR   = LPSPI_CCR_SCKPCS(4)
				  |LPSPI_CCR_PCSSCK(4)
				  |LPSPI_CCR_DBT(8)
				  |LPSPI_CCR_SCKDIV(8);   	/* Clock dividers based on prescaled func'l clk of 100 nsec 	*/
											/* SCKPCS=4: SCK to PCS delay = 4+1 = 5 (500 nsec) 			*/
											/* PCSSCK=4: PCS to SCK delay = 9+1 = 10 (1 usec) 				*/
											/* DBT=8: Delay between Transfers = 8+2 = 10 (1 usec) 			*/
											/* SCKDIV=8: SCK divider =8+2 = 10 (1 usec: 1 MHz baud rate) 	*/

  LPSPI1->FCR   = LPSPI_FCR_TXWATER(3);   	/* RXWATER=0: Rx flags set when Rx FIFO >0 	*/
                                			/* TXWATER=3: Tx flags set when Tx FIFO <= 3 	*/

  LPSPI1->CR    = LPSPI_CR_MEN_MASK
		  	  	  |LPSPI_CR_DBGEN_MASK;   	/* Enable module for operation 			*/
											/* DBGEN=1: module enabled in debug mode 	*/
											/* DOZEN=0: module enabled in Doze mode 	*/
											/* RST=0: Master logic not reset 			*/
											/* MEN=1: Module is enabled 				*/
}

void LPSPI1_transmit_16bits (uint16_t send)
{
  while((LPSPI1->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT==0);
                                   /* Wait for Tx FIFO available 	*/
  LPSPI1->TDR = send;              /* Transmit data 				*/
  LPSPI1->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
}

uint16_t LPSPI1_receive_16bits (void)
{
  uint16_t recieve = 0;

  while((LPSPI1->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==0);
                                   /* Wait at least one RxFIFO entry 	*/
  recieve= LPSPI1->RDR;            /* Read received data 				*/
  LPSPI1->SR |= LPSPI_SR_RDF_MASK; /* Clear RDF flag 					*/
  return recieve;                  /* Return received data 			*/
}

/*!
* @brief Initialize the SPI interface to use.
*/
void LPSPI0_init_master (void)
{
	/* Configure Clock for LPSPI0 */
	PCC -> PCCn[PCC_LPSPI0_INDEX] &= ~PCC_PCCn_CGC_MASK;  	/* CGC = 0 Clock disabled to modify PCS */
	PCC -> PCCn[PCC_LPSPI0_INDEX] &= ~PCC_PCCn_PCS_MASK;  	/* Clear PCS field temporarily */
	PCC -> PCCn[PCC_LPSPI0_INDEX] |= PCC_PCCn_PCS(6);     	/* PCS = 6 SPLLDIV2_CLK is selected */
	PCC -> PCCn[PCC_LPSPI0_INDEX] |= PCC_PCCn_CGC_MASK;   	/* CGC = 1 Clock enabled for LPSPI0 */

	/* Configure as master mode */
	LPSPI0 -> CFGR1 |= LPSPI_CFGR1_MASTER_MASK; 			/* MASTER = 1 Master mode */

	LPSPI0 -> TCR = LPSPI_TCR_PRESCALE(2)                   /* Divide by 4, = 10 MHz */
				  | LPSPI_TCR_FRAMESZ(7);    				/* Configure TCR, FRAMESZ = 7 Frame size is 8bit */

	LPSPI0 -> CCR |= LPSPI_CCR_SCKDIV(8)   					/* SCKDIV = 8 SCK cycle is 8+2 = 10 cycles of 10MHz thus 1MHz */
				  |  LPSPI_CCR_SCKPCS(4)   					/* SCKPCS = 4 SCK to PCS Delay is (4 + 1) = 5 baud clock cycles */
				  |  LPSPI_CCR_PCSSCK(4)   					/* PCSSCK = 4 PCS to SCK Delay is (4 + 1) = 5 baud clock cycles */
				  |  LPSPI_CCR_DBT(8);      				/* DBT = 8 Delay Between Transfers is (8 + 2) = 10 baud clock cycles */

	LPSPI0 -> CR |= LPSPI_CR_MEN_MASK;      				/* MEN = 1 Module is enabled */

	LPSPI0 -> FCR &= ~LPSPI_FCR_RXWATER_MASK;  				/* RXWATER = 0 The Receive Data Flag is set whenever the number of words in the receive FIFO is greater than 0 */
	LPSPI0 -> FCR &= ~LPSPI_FCR_TXWATER_MASK;  				/* TXWATER = 0 The Transmit Data Flag is set whenever the number of words in the transmit FIFO is equal or less than 0 */

}

/*!
* @brief Transfer 8-bits data by SPI.
*
* @param[uint8_t data] 8-bits data.
*
* @return[uint8_t receive] Return received data.
*/
uint8_t LPSPI0_write_8bits (uint8_t data)
{
	uint8_t receive;

	/* Wait for Tx FIFO available */
	while((LPSPI0 -> SR & LPSPI_SR_TDF_MASK) >> LPSPI_SR_TDF_SHIFT == 0);

	LPSPI0 -> TDR = data;              					/* Transmit data */
	LPSPI0 -> SR |= LPSPI_SR_TDF_MASK; 					/* Clear TDF flag */

	/* Wait at least one RxFIFO entry */
	while((LPSPI0 -> SR & LPSPI_SR_RDF_MASK) >> LPSPI_SR_RDF_SHIFT == 0);

	receive = LPSPI0 -> RDR;            				/* Read received data */
	LPSPI0 -> SR |= LPSPI_SR_RDF_MASK; 					/* Clear RDF flag */

	return receive; 									/* Return received data */
}

/*!
* @brief Function RS pin goes high.
*/
void LCD_RS_data (void)
{
	PTD -> PSOR = (1 << 14);    						/* PTD14: RS goes high */
}

/*!
* @brief Function RS pin goes low (CMD).
*/
void LCD_RS_command (void)
{
	PTD -> PCOR = (1 << 14);    						/* PTD14: RS goes low  */
}

/*!
* @brief Function SPI CS Active.
*/
void LCD_CS_low (void)
{
	PTB -> PCOR = (1 << 5);    							/* PTB5: CS goes low  */
}

/*!
* @brief Function SPI CS Inactive.
*/
void LCD_CS_high (void)
{
	PTB -> PSOR = (1 << 5);    							/* PTB5: CS goes high  */
}

/*!
* @brief Function SPI CS Active.
*/
void STMPE610_CS_low (void)
{
	PTD -> PCOR = (1 << 13);    						/* PTD13: CS goes low  */
}

/*!
* @brief Function SPI CS Inactive.
*/
void STMPE610_CS_high (void)
{
	PTD -> PSOR = (1 << 13);    						/* PTD13: CS goes high */
}
