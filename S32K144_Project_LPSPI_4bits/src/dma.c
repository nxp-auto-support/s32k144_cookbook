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

#include "device_registers.h"							/* include peripheral declarations */
#include "dma.h"

uint8_t TCD0_Source[] = {"Hello World"};				/* TCD 0 source (11 byte string) */
uint8_t volatile TCD0_Destination = 0;             		/* TCD 0 destination (1 byte) */


/*!
* @brief DMA Initialization. Select LPSPI1 Tx channel source.
*/
void DMA_init (void)
{
	/* Enable clock for DMAMUX */
	PCC -> PCCn[PCC_DMAMUX_INDEX] = PCC_PCCn_CGC_MASK;

	/* DMA Channel Source Select - always enabled and PIT CH1 Enable */
	DMAMUX -> CHCFG[0] = DMAMUX_CHCFG_SOURCE(EDMA_REQ_LPSPI1_TX) | DMAMUX_CHCFG_ENBL_MASK;
}


/*!
 * @brief Configure the TCD of the DMA to allow sending each character (byte) of the
 *        source string to one single memory location defined as the destination address.
 */
void DMA_TCD_init (void)
{
	/* TCD0: Transfers string to a single memory location */

	/* Source Address */
	DMA -> TCD[0].SADDR = DMA_TCD_SADDR_SADDR((uint32_t volatile) &TCD0_Source);

	DMA -> TCD[0].SOFF  = DMA_TCD_SOFF_SOFF(4);   					/* Source address add 4 bytes after transfers */

	DMA -> TCD[0].ATTR  = DMA_TCD_ATTR_SMOD(0)   					/* Source modulo feature not used */
						| DMA_TCD_ATTR_SSIZE(0)  					/* Source read 2^0 = 1 byte per transfer */
                        | DMA_TCD_ATTR_DMOD(0)   					/* Destination modulo feature not used */
                        | DMA_TCD_ATTR_DSIZE(0);  					/* Destination write 2^0 = 1 byte per transfer */

	DMA -> TCD[0].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(1); 		/* 1 byte per transfer request. Minor loop */

	DMA -> TCD[0].SLAST = DMA_TCD_SLAST_SLAST(-11); 				/* Source address change after major loop */

	/* Destination Address */
	DMA -> TCD[0].DADDR = DMA_TCD_DADDR_DADDR((uint32_t volatile) &TCD0_Destination);

	DMA -> TCD[0].DOFF = DMA_TCD_DOFF_DOFF(0);     					/* No destination address offset after transfer */

	DMA -> TCD[0].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(11)	/* 11 minor loop iterations */
                                | DMA_TCD_CITER_ELINKNO_ELINK(0);   /* No minor loop channel linking */

	DMA -> TCD[0].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(0); 			/* No destination change after major loop */

	DMA -> TCD[0].CSR = DMA_TCD_CSR_START(0)         				/* Clear START status flag. Channel is not explicitly started */
                      | DMA_TCD_CSR_INTMAJOR(0)   					/* No IRQ after major loop */
                      | DMA_TCD_CSR_INTHALF(0)       				/* No IRQ after 1/2 major loop */
                      | DMA_TCD_CSR_DREQ(1)          				/* Disable channel after major loop */
                      | DMA_TCD_CSR_ESG(0)           				/* Disable Scatter Gather */
                      | DMA_TCD_CSR_MAJORELINK(0)    				/* No major loop channel linking */
                      | DMA_TCD_CSR_ACTIVE(0)        				/* Clear ACTIVE status flag */
                      | DMA_TCD_CSR_DONE(0)          				/* Clear DONE status flag */
                      | DMA_TCD_CSR_MAJORLINKCH(0)   				/* No channel-to-channel linking */
                      | DMA_TCD_CSR_BWC(0);           				/* No eDMA stalls after read/write */

	DMA -> TCD[0].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(11)  	/* Initial iteration count */
                                | DMA_TCD_BITER_ELINKNO_ELINK(0);   /* No minor loop channel linking */
}


/*!
* @brief Save the TCD configurations in the TCD_t type variable with the corresponding information.
*        Depending on the inputs it may configure the TCD to transfer a string to a string or a variable to a string or a string to a variable.
*
* @param [uint32_t * buff_source]      Pointer to the direction of the Source Address
* @param [uint32_t * buff_destination] Pointer to the direction of the Destination Address
* @param [uint32_t size]               Amount of Minor loops required
* @param [TCD_t * TCDm]                Pointer to the TCDm index where the TCD configuration will be saved
*/
void DMA_TCDm_config (uint32_t * buffer_source, uint32_t * buffer_destination, uint32_t size, TCD_t * TCDm)
{
	/* TCDm: Save TCD configurations */

	/* Source Address */
	TCDm -> SADDR = DMA_TCD_SADDR_SADDR((uint32_t) &buffer_source[0]);

	TCDm -> SOFF  = DMA_TCD_SOFF_SOFF(4);   						/* Source address add 4 bytes after transfers */

	TCDm -> ATTR  = DMA_TCD_ATTR_SMOD(0)   							/* Source modulo feature not used */
				  |	 DMA_TCD_ATTR_SSIZE(2)  						/* Source read 2^2 = 4 bytes per transfer */
				  |	DMA_TCD_ATTR_DMOD(0)   							/* Destination modulo feature not used */
				  | DMA_TCD_ATTR_DSIZE(2);  						/* Destination write 2^2 = 4 bytes per transfer */

	TCDm -> NBYTES_MLOFFNO = DMA_TCD_NBYTES_MLNO_NBYTES(4); 		/* 4 bytes per transfer request. Minor loop */

	TCDm -> SLAST = DMA_TCD_SLAST_SLAST(0); 						/* No source address change after major loop */

	/* Destination Address */
	TCDm -> DADDR = DMA_TCD_DADDR_DADDR((uint32_t) &buffer_destination[0]);

	TCDm -> DOFF = DMA_TCD_DOFF_DOFF(0);     						/* No destination address offset after transfer */

	TCDm -> CITER_ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(size)  		/* Define the quantity of minor loop iterations */
	                      | DMA_TCD_CITER_ELINKNO_ELINK(0);   		/* No minor loop channel linking */

	TCDm -> DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(0); 				/* No destination change after major loop */

	TCDm -> CSR = DMA_TCD_CSR_START(0)       						/* Clear START status flag. Channel is not explicitly started */
	            | DMA_TCD_CSR_INTMAJOR(0)      						/* No IRQ after major loop */
	            | DMA_TCD_CSR_INTHALF(0)       						/* No IRQ after 1/2 major loop */
	            | DMA_TCD_CSR_DREQ(1)          						/* Disable channel after major loop */
	            | DMA_TCD_CSR_ESG(0)           						/* Disable Scatter Gather */
	            | DMA_TCD_CSR_MAJORELINK(0)    						/* No major loop channel linking */
	            | DMA_TCD_CSR_ACTIVE(0)        						/* Clear ACTIVE status flag */
	            | DMA_TCD_CSR_DONE(0)          						/* Clear DONE status flag */
	            | DMA_TCD_CSR_MAJORLINKCH(0)   						/* No channel-to-channel linking */
	            | DMA_TCD_CSR_BWC(0);          	 					/* No eDMA stalls after read/write */

	TCDm -> BITER_ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(size)  		/* Initial iteration count */
	                      | DMA_TCD_BITER_ELINKNO_ELINK(0);    		/* No minor loop channel linking */

}


/*!
* @brief: Fill out the TCD of the desired DMA channel using the configuration
*         saved in memory of the TCDm index selected.
*
* @param [uint8_t channel] DMA channel where the TCD configuration will be applied
* @param [TCD_t * TCDm]    Pointer to the TCDm index which contains the TCD configuration
*/
void DMA_TCD_Push (uint8_t channel, TCD_t * TCDm)
{
	/* Fill out the TCD of the DMA Channel */

	DMA -> TCD[channel].SADDR         = TCDm -> SADDR;
	DMA -> TCD[channel].SOFF          = TCDm -> SOFF;
	DMA -> TCD[channel].ATTR          = TCDm -> ATTR;

	DMA -> TCD[channel].NBYTES.MLNO   = TCDm -> NBYTES_MLOFFNO;
	DMA -> TCD[channel].SLAST         = TCDm -> SLAST;

	DMA -> TCD[channel].DADDR         = TCDm -> DADDR;
	DMA -> TCD[channel].DOFF          = TCDm -> DOFF;
	DMA -> TCD[channel].CITER.ELINKNO = TCDm -> CITER_ELINKNO;

	DMA -> TCD[channel].DLASTSGA      = TCDm -> DLASTSGA;
	DMA -> TCD[channel].CSR           = TCDm -> CSR;

	DMA -> TCD[channel].BITER.ELINKNO = TCDm -> BITER_ELINKNO;
}
