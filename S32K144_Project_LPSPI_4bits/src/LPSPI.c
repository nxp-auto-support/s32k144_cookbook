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

#include "device_registers.h"	  						/* include peripheral declarations */
#include "LPSPI.h"


/*!
* @brief LPSPI1 Initialization. Master Mode.
*/
void LPSPI1_init_master (void)
{
	/* LPSPI1 Clocking */
	PCC -> PCCn[PCC_LPSPI1_INDEX] = 0;          		/* Disable clocks to modify PCS */
	PCC -> PCCn[PCC_LPSPI1_INDEX] = PCC_PCCn_PR_MASK	/* Peripheral is present (default) */
								  | PCC_PCCn_CGC_MASK	/* Enable Clock */
								  | PCC_PCCn_PCS(6);	/* Clock Source = SPLL_DIV2 (40 MHz) */

	/* LPSPI1 Initialization */
	LPSPI1 -> CR    = 0x00000000;   					/* Disable module for configuration */
	LPSPI1 -> IER   = 0x00000000;   					/* Interrupts not used */
	LPSPI1 -> DER   = 0x00000000;   					/* DMA not used */

	LPSPI1 -> CFGR0 = 0x00000000;   					/* RDM0 = 0: Received data is store in the FIFO as normal */
                                						/* CIRFIFO = 0: Circular FIFO is disabled */
                                						/* HRSEL, HRPOL, HREN = 0: Host request disabled */

	/* Configurations: Master Mode */
	LPSPI1 -> CFGR1 = LPSPI_CFGR1_MASTER_MASK;  		/* MASTER = 1: Master Mode */
														/* PCSCFG = 0: PCS[3:2] are enabled */
														/* OUTCFG = 0: Output data retains last value when CS negated */
														/* PINCFG = 0: SIN is input, SOUT is output */
														/* MATCFG = 0: Match disabled */
														/* PCSPOL = 0: PCS is active low */
														/* NOSTALL = 0: Stall if Tx FIFO is empty or Rx FIFO is full */
														/* AUTOPCS = 0: Does not apply for master mode */
														/* SAMPLE = 0: Input data sampled on SCK edge */

	/* Transmit Command: PCS3, 16 bits, prescaler functional clock by 4 */
	LPSPI1 -> TCR = LPSPI_TCR_CPHA_MASK					/* CPHA = 1: Change data on SCK leading, capture on trailing edge */
				  | LPSPI_TCR_PRESCALE(2)				/* PRESCALE = 2: Functional clock divided by 2^2 = 4 */
				  | LPSPI_TCR_PCS(3)					/* PCS = 3: Transfer using PCS3 */
				  | LPSPI_TCR_FRAMESZ(15);  			/* FRAMESZ = 15: # bits in frame = 15 + 1 = 16 */
														/* CPOL = 0: SCK inactive state is low */
														/* LSBF = 0: Data is transfered MSB first */
														/* BYSW = 0: Byte swap disabled */
														/* CONT, CONTC = 0: Continuous transfer disabled */
														/* RXMSK = 0: Normal transfer, Rx data stored in Rx FIFO */
														/* TXMSK = 0: Normal transfer, data loaded from Tx FIFO */
														/* WIDTH = 0: Single bit transfer */

	/* Clock dividers based on prescaled functional clock of 100 nsec (40 MHz / 4)*/
	LPSPI1 -> CCR = LPSPI_CCR_SCKPCS(4)					/* SCKPCS = 4: SCK to PCS delay = 4 + 1 = 5 (500 nsec) */
				  | LPSPI_CCR_PCSSCK(4)					/* PCSSCK = 4: PCS to SCK delay = 4 + 1 = 5 (500 nsec) */
				  | LPSPI_CCR_DBT(8)					/* DBT = 8: Delay between transfers = 8 + 2 = 10 (1 usec) */
				  | LPSPI_CCR_SCKDIV(8);   				/* SCKDIV = 8: SCK divider = 8 + 2 = 10 (1 usec) */

	LPSPI1 -> FCR = LPSPI_FCR_TXWATER(3);   			/* TXWATER = 3: Tx flags set when Tx FIFO <= 3 */
														/* RXWATER = 0: Rx flags set when Rx FIFO > 0 */

	LPSPI1 -> CR = LPSPI_CR_MEN_MASK					/* Enable module for operation */
		  	  	 | LPSPI_CR_DBGEN_MASK;  				/* DBGEN = 1: Module enabled in debug mode */
														/* DOZEN = 0: Module enabled in doze mode */
														/* RST = 0: Master logic not reset */
														/* MEN = 1: Module is enabled */
}


/*!
* @brief Enable SPI 4 bit mode.
*/
void LPSPI1_4bitMode_enable (void)
{
	LPSPI1 -> DER = LPSPI_DER_TDDE_MASK;       			/* Transmit data DMA request enable */

	/* Configurations: Master Mode */
	LPSPI1 -> CFGR1 |= LPSPI_CFGR1_MASTER_MASK			/* MASTER = 1: Master Mode */
			  	  	|  LPSPI_CFGR1_NOSTALL_MASK			/* NOSTALL = 1: No Stall if Tx FIFO is empty or Rx FIFO is full */
					|  LPSPI_CFGR1_PCSCFG_MASK;  		/* PCSCFG = 1: PCS[3:2] are enabled. Half - duplex 4-bit transfer */
														/* OUTCFG = 0: Output data retains last value when CS negated */
														/* PINCFG = 0: SIN is input, SOUT is output */
														/* MATCFG = 0: Match disabled */
														/* PCSPOL = 0: PCS is active low */
														/* AUTOPCS = 0: Does not apply for master mode */
														/* SAMPLE = 0: Input data sampled on SCK edge */
}


/*!
* @brief Transfer 16-bits data by SPI.
*
* @param [uint16_t send] 16-bits data to be sent
*/
void LPSPI1_transmit_16bits (uint16_t send)
{
	/* Wait for Tx FIFO available */
	while((LPSPI1 -> SR & LPSPI_SR_TDF_MASK) >> LPSPI_SR_TDF_SHIFT == 0);

	LPSPI1 -> TDR = send;              					/* Transmit data */
	LPSPI1 -> SR |= LPSPI_SR_TDF_MASK; 					/* Clear TDF flag */
}


/*!
* @brief Receive 16-bits data by SPI.
*
* @return [uint16_t receive] 16-bits data to be received
*/
uint16_t LPSPI1_receive_16bits (void)
{
  uint16_t receive = 0;

  /* Wait at least one Rx FIFO entry */
  while((LPSPI1->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==0);

  receive = LPSPI1 -> RDR;            					/* Read received data */
  LPSPI1 -> SR |= LPSPI_SR_RDF_MASK; 					/* Clear RDF flag */

  return receive;                  						/* Return received data */
}
