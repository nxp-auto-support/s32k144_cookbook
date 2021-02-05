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

#include "device_registers.h"	/* include peripheral declarations */
#include "dma.h"

uint8_t TCD0_Source[] = {"Hello World"};	/*< TCD 0 source (11 byte string) 	*/
uint8_t volatile TCD0_Dest = 0;             /*< TCD 0 destination (1 byte) 	*/

void DMA_init(void)
{
 /* This is an initialization place holder for:   											*/
 /* 1. Enabling DMA MUX clock PCC_PCCn[PCC_DMAMUX_INDEX] (not needed when START bit used) 	*/
 /* 2. Enabling desired channels by setting ERQ bit (not needed when START bit used) 		*/
}

void DMA_TCD_init(void)
{
	/*!
	 * TCD0: Transfers string to a single memory location
	 * ===================================================
	 */
  DMA->TCD[0].SADDR        = DMA_TCD_SADDR_SADDR((uint32_t volatile) &TCD0_Source); /* Source Address. */
  DMA->TCD[0].SOFF         = DMA_TCD_SOFF_SOFF(1);   /* Src. addr add 1 byte after Transfers   */
  DMA->TCD[0].ATTR         = DMA_TCD_ATTR_SMOD(0)  | /* Src. modulo feature not used */
                             DMA_TCD_ATTR_SSIZE(0) | /* Src. read 2**0 =1 byte per transfer */
                             DMA_TCD_ATTR_DMOD(0)  | /* Dest. modulo feature not used */
                             DMA_TCD_ATTR_DSIZE(0);  /* Dest. write 2**0 =1 byte per trans. */

  DMA->TCD[0].NBYTES.MLNO  = DMA_TCD_NBYTES_MLNO_NBYTES(1); /* Transfer 1 byte /minor loop */
  DMA->TCD[0].SLAST        = DMA_TCD_SLAST_SLAST(-11); 		/* Src addr change after major loop */

  DMA->TCD[0].DADDR        = DMA_TCD_DADDR_DADDR((uint32_t volatile) &TCD0_Dest);	/* Destination Address. */
  DMA->TCD[0].DOFF         = DMA_TCD_DOFF_DOFF(0);     			/* No dest adr offset after transfer */
  DMA->TCD[0].CITER.ELINKNO= DMA_TCD_CITER_ELINKNO_CITER(11) | 	/* 11 minor loop iterations */
                             DMA_TCD_CITER_ELINKNO_ELINK(0);   	/* No minor loop chan link */

  DMA->TCD[0].DLASTSGA     = DMA_TCD_DLASTSGA_DLASTSGA(0); /* No dest chg after major loop */
  DMA->TCD[0].CSR          = DMA_TCD_CSR_START(0)       |  /* Clear START status flag */
                             DMA_TCD_CSR_INTMAJOR(0)    |  /* No IRQ after major loop */
                             DMA_TCD_CSR_INTHALF(0)     |  /* No IRQ after 1/2 major loop */
                             DMA_TCD_CSR_DREQ(1)        |  /* Disable chan after major loop */
                             DMA_TCD_CSR_ESG(0)         |  /* Disable Scatter Gather */
                             DMA_TCD_CSR_MAJORELINK(0)  |  /* No major loop chan link */
                             DMA_TCD_CSR_ACTIVE(0)      |  /* Clear ACTIVE status flag */
                             DMA_TCD_CSR_DONE(0)        |  /* Clear DONE status flag */
                             DMA_TCD_CSR_MAJORLINKCH(0) |  /* Chan # if major loop ch link */
                             DMA_TCD_CSR_BWC(0);           /* No eDMA stalls after R/W */
  DMA->TCD[0].BITER.ELINKNO= DMA_TCD_BITER_ELINKNO_BITER(11) |  /* Initial iteration count */
                             DMA_TCD_BITER_ELINKNO_ELINK(0);    /* No minor loop chan link */
}


