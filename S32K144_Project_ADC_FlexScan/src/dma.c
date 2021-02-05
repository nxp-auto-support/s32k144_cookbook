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
uint8_t volatile TCD_LC_Dest[11];			/*< Linking Channel destination (11 byte string) */
uint32_t volatile ADC_SC1A_CH[3] = {8,9,12}; /*< Array to set up the external channels to measure: */
											/*< 8 -> PTB13, 9-> PTB14, 12-> Potentiometer			*/
uint32_t volatile ADC_Results[16];			/*< Destination of the results of each ADC sample made */

void DMA_init(void)
{
 /* This is an initialization place holder for:   											*/
 /* 1. Enabling DMA MUX clock PCC_PCCn[PCC_DMAMUX_INDEX] (not needed when START bit used) 	*/
 /* 2. Enabling desired channels by setting ERQ bit (not needed when START bit used) 		*/
}

/*!
 * TCD0: Transfers string to a single memory location
 * ===================================================
 * This function configures the TCD of the DMA to allow it to send each
 * character of the source string to one single memory location defined as the
 * destination address.
 */
void DMA_TCD_init(void)
{

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

void DMA_SG_init(void){
	PCC->PCCn[PCC_DMAMUX_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for DMAMUX */
	DMAMUX->CHCFG[0] = DMAMUX_CHCFG_SOURCE(EDMA_REQ_DMAMUX_ALWAYS_ENABLED0) | DMAMUX_CHCFG_ENBL_MASK;	/* Keep the trigger always enabled for DMACH0 and turn on Enable CH0 */
}

/*!
* Save TCD configurations
* ===================================================
* This function saves the TCD configurations in the TCD_t type variable
* with the according information.
* Depending on the inputs it may configure the TCD to transfer a string
* to a string or a variable to a string or a string to a variable.
*
* @param[uint32_t * buff_source] Pointer to the direction Source Address
* @param[uint8_t SOFF] Amount of bytes added to Source Address after transfer
* @param[uint32_t * buff_dest] Pointer to the direction Destination Address
* @param[uint8_t DOFF] Amount of bytes added to Direction Address after transfer
* @param[uint32_t size] Amount of Minor loops required
* @param[TCD_t * TCDm] Pointer to the TCDm index where the TCD configuration will be saved
*
*/
void DMA_TCDm_config(uint32_t * buff_source, uint8_t SOFF, uint32_t * buff_dest, uint8_t DOFF, uint32_t size, TCD_t * TCDm )
{
	TCDm->SADDR 		= DMA_TCD_SADDR_SADDR((uint32_t) buff_source); /* Source Address */
	TCDm->SOFF  		= DMA_TCD_SOFF_SOFF(SOFF);   /* Src. addr add 1 byte after Transfers   */
	TCDm->ATTR  		= DMA_TCD_ATTR_SMOD(0)  | /* Src. modulo feature not used */
					      DMA_TCD_ATTR_SSIZE(0) | /* Src. read 2**0 =1 byte per transfer */
						  DMA_TCD_ATTR_DMOD(0)  | /* Dest. modulo feature not used */
					      DMA_TCD_ATTR_DSIZE(0);  /* Dest. write 2**0 =1 byte per trans. */

	TCDm->NBYTES_MLOFFNO= DMA_TCD_NBYTES_MLNO_NBYTES(1); /* Transfer 1 byte /minor loop */
	TCDm->SLAST         = DMA_TCD_SLAST_SLAST(-(SOFF*size)); 		/* No Src addr change after major loop */

	TCDm->DADDR        	= DMA_TCD_DADDR_DADDR((uint32_t) buff_dest);	/* Destination Address. */
	TCDm->DOFF         	= DMA_TCD_DOFF_DOFF(DOFF);     			/* No dest adr offset after transfer */
	TCDm->CITER_ELINKNO	= DMA_TCD_CITER_ELINKNO_CITER(size) | 	/* size defines the quantity of minor loop iterations */
	                      DMA_TCD_CITER_ELINKNO_ELINK(0);   	/* No minor loop chan link */

	TCDm->DLASTSGA     	= DMA_TCD_DLASTSGA_DLASTSGA(-(DOFF*size)); /* Dest chg depends on DOFF after major loop */
	TCDm->CSR           = DMA_TCD_CSR_START(0)       |  /* Clear START status flag */
	                      DMA_TCD_CSR_INTMAJOR(1)    |  /* IRQ after major loop */
	                      DMA_TCD_CSR_INTHALF(0)     |  /* No IRQ after 1/2 major loop */
	                      DMA_TCD_CSR_DREQ(1)        |  /* Disable chan after major loop */
	                      DMA_TCD_CSR_ESG(0)         |  /* Disable Scatter Gather */
	                      DMA_TCD_CSR_MAJORELINK(0)  |  /* No major loop chan link */
	                      DMA_TCD_CSR_ACTIVE(0)      |  /* Clear ACTIVE status flag */
	                      DMA_TCD_CSR_DONE(0)        |  /* Clear DONE status flag */
	                      DMA_TCD_CSR_MAJORLINKCH(0) |  /* Chan # if major loop ch link */
	                      DMA_TCD_CSR_BWC(0);           /* No eDMA stalls after R/W */
	TCDm->BITER_ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(size) |  /* size defines the initial iteration count */
	                      DMA_TCD_BITER_ELINKNO_ELINK(0);    /* No minor loop chan link */

}

/*!
* Fill out the TCD of the DMA Channel
* ===================================================
* Fill out the TCD of the desired DMA channel using the
* configuration saved in memory of the TCDm index selected.
*
* @param[uint8_t ch] DMA channel where you want to apply the TCD configuration
* @param[TCD_t * TCDm] Pointer to the TCDm index which contains the TCD configuration to be applied.
*
*/
void DMA_TCD_Push(uint8_t ch, TCD_t * TCDm )
{
   DMA->TCD[ch].SADDR        = TCDm->SADDR;
   DMA->TCD[ch].SOFF         = TCDm->SOFF;
   DMA->TCD[ch].ATTR         = TCDm->ATTR;


   DMA->TCD[ch].NBYTES.MLNO  = TCDm->NBYTES_MLOFFNO;
   DMA->TCD[ch].SLAST        = TCDm->SLAST;

   DMA->TCD[ch].DADDR        = TCDm->DADDR;
   DMA->TCD[ch].DOFF         = TCDm->DOFF;
   DMA->TCD[ch].CITER.ELINKNO= TCDm->CITER_ELINKNO;


   DMA->TCD[ch].DLASTSGA     = TCDm->DLASTSGA;
   DMA->TCD[ch].CSR          = TCDm->CSR;

   DMA->TCD[ch].BITER.ELINKNO= TCDm->BITER_ELINKNO;
}

/*! Configuration of the DMA for CAN Node 2
 * 	=====================================================
 * 	Enable DMA channel 3 for data transfer from 4 channels of the ADC0
 * 	to a ADC_Results using the DMA, the DMA transfer call starts when
 * 	the flag COCO from any channel is enable.
 *
 * 	@param[uint32_t Destination[4]] 4 size array that will be defined as the Destination Address
 *
 */
void DMA_Config(uint32_t Destination[4]){
	SIM->PLATCGC |= SIM_PLATCGC_CGCDMA_MASK;			/* DMA Clock Gating Control Enable */

	PCC->PCCn[PCC_DMAMUX_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable DMA Clock */

	DMAMUX->CHCFG[3] &=~ DMAMUX_CHCFG_ENBL_MASK;        /* Disabling the DMA channel 3 */
	DMAMUX->CHCFG[3] |= DMAMUX_CHCFG_SOURCE(42);        /* ADC0 COCO is the source of the DMA channel 3 */
	DMAMUX->CHCFG[3] |= DMAMUX_CHCFG_ENBL_MASK;         /* Enable the DMA channel 3 */

	/* Configure TCD*/
	//Source
	DMA->TCD[3].SADDR = (uint32_t)&ADC0->R[4];		/* Data Source */
	DMA->TCD[3].SOFF = DMA_TCD_SOFF_SOFF(2);		/* Offset Source 1 Byte */
	DMA->TCD[3].SLAST = DMA_TCD_SLAST_SLAST(-20);	/* An adjust of -5 is needed because are 4 channels*/

	//Destination
	DMA->TCD[3].DADDR = (uint32_t)&Destination[0];       	/* Data Destination */
	DMA->TCD[3].DOFF = DMA_TCD_DOFF_DOFF(2);   				/* Offset Source 1 Byte */
	DMA->TCD[3].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-20);	/* An adjust of -5 is needed */

	DMA->TCD[3].ATTR = DMA_TCD_ATTR_SSIZE(1)|	/* 16 bits transfer from Source */
					   DMA_TCD_ATTR_DSIZE(1);	/* 16 bits receive to Destination */

	DMA->TCD[3].NBYTES.MLOFFNO = DMA_TCD_NBYTES_MLOFFYES_NBYTES(2);   	/* Minor Byte Transfer Count is 2-bytes */
	DMA->TCD[3].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(10);	/* Current Major Iteration Count is 1 */
	DMA->TCD[3].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(10);

	DMA->TCD[3].CSR = DMA_TCD_CSR_MAJORELINK_MASK|	/* The channel-to-channel linking is enable */
					  DMA_TCD_CSR_MAJORLINKCH(1);   /* The channel is not explicitly started*/

	DMA->ERQ |= DMA_ERQ_ERQ3_MASK;    /* The DMA request signal for CH3 is enabled */

}

/*!
 * DMAMUX initialization for it to always request in the desired channel (Channel 0)
 */

void DMAMUX_LC_init(void){
	PCC->PCCn[PCC_DMAMUX_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for DMAMUX */
	DMAMUX->CHCFG[0] = DMAMUX_CHCFG_SOURCE(EDMA_REQ_DMAMUX_ALWAYS_ENABLED0) | DMAMUX_CHCFG_ENBL_MASK;	/* Keep the trigger always enabled for DMACH0 and turn on Enable CH0 */
}


/*!
 * DMA Linking Channel Feature
 * ===================================================
 * Set up DMA TCD 0 to start filling a string and generate a link to channel 1 after
 * its major loop has ended, set up DMA TCD 1 to finish filling up the string with the
 * remaining amount of bytes.
 *
 */
void DMA_TCD_LC_Config(void){
	  DMA->TCD[0].SADDR        = DMA_TCD_SADDR_SADDR((uint32_t volatile) &TCD0_Source[0]); /* Source Address. */
	  DMA->TCD[0].SOFF         = DMA_TCD_SOFF_SOFF(1);   /* Src. addr add 1 byte after Transfers   */
	  DMA->TCD[0].ATTR         = DMA_TCD_ATTR_SMOD(0)  | /* Src. modulo feature not used */
	                             DMA_TCD_ATTR_SSIZE(0) | /* Src. read 2**0 =1 byte per transfer */
	                             DMA_TCD_ATTR_DMOD(0)  | /* Dest. modulo feature not used */
	                             DMA_TCD_ATTR_DSIZE(0);  /* Dest. write 2**0 =1 byte per trans. */

	  DMA->TCD[0].NBYTES.MLNO  = DMA_TCD_NBYTES_MLNO_NBYTES(1); /* Transfer 1 byte /minor loop */
	  DMA->TCD[0].SLAST        = DMA_TCD_SLAST_SLAST(-6); 		/* Src addr change -6 bytes after major loop */

	  DMA->TCD[0].DADDR        = DMA_TCD_DADDR_DADDR((uint32_t volatile) &TCD_LC_Dest[0]);	/* Destination Address. */
	  DMA->TCD[0].DOFF         = DMA_TCD_DOFF_DOFF(1);     			/* Dest. adr add 1 byte after transfers */
	  DMA->TCD[0].CITER.ELINKYES= DMA_TCD_CITER_ELINKYES_CITER_LE(6)| 	/* 6 minor loop iterations */
	  	  	  	  	  	  	  	 DMA_TCD_CITER_ELINKYES_ELINK_MASK|	/* Enable Linking Channel after Minor Loop */
			  	  	  	  	  	 DMA_TCD_CITER_ELINKYES_LINKCH(1);  /* Link to channel 1 after minor loop */

	  DMA->TCD[0].DLASTSGA     = DMA_TCD_DLASTSGA_DLASTSGA(-6); /* Dest. chg -6 bytes after major loop */
	  DMA->TCD[0].CSR          = DMA_TCD_CSR_START(0)       |  /* Clear START status flag */
	                             DMA_TCD_CSR_INTMAJOR(0)    |  /* No IRQ after major loop */
	                             DMA_TCD_CSR_INTHALF(0)     |  /* No IRQ after 1/2 major loop */
	                             DMA_TCD_CSR_DREQ(1)        |  /* Disable chan after major loop */
	                             DMA_TCD_CSR_ESG(0)         |  /* Disable Scatter Gather */
	                             DMA_TCD_CSR_MAJORELINK(0)  |  /* Activate major loop chan link */
	                             DMA_TCD_CSR_ACTIVE(0)      |  /* Clear ACTIVE status flag */
	                             DMA_TCD_CSR_DONE(0)        |  /* Clear DONE status flag */
	                             DMA_TCD_CSR_MAJORLINKCH(0) |  /* Chan # if major loop ch link (Channel 1 in this case) */
	                             DMA_TCD_CSR_BWC(0);           /* No eDMA stalls after R/W */
	  DMA->TCD[0].BITER.ELINKYES= DMA_TCD_BITER_ELINKYES_BITER(6) |  /* Initial iteration count */
			  	  	  	  	  	 DMA_TCD_BITER_ELINKYES_ELINK_MASK|   /*  Minor loop Linking Channel */
	  	  	  	  	  	  	  	 DMA_TCD_BITER_ELINKYES_LINKCH(1);	/* Minor Loop to CH 1 */


	  /*Set up DMA TCD CH 1 */

	  DMA->TCD[1].SADDR        = DMA_TCD_SADDR_SADDR((uint32_t volatile) &TCD0_Source[6]); /* Source Address. */
	  DMA->TCD[1].SOFF         = DMA_TCD_SOFF_SOFF(1);   /* Src. addr add 1 byte after Transfers   */
	  DMA->TCD[1].ATTR         = DMA_TCD_ATTR_SMOD(0)  | /* Src. modulo feature not used */
			  	  	  	  	  	  DMA_TCD_ATTR_SSIZE(0) | /* Src. read 2**0 =1 byte per transfer */
								  DMA_TCD_ATTR_DMOD(0)  | /* Dest. modulo feature not used */
								  DMA_TCD_ATTR_DSIZE(0);  /* Dest. write 2**0 =1 byte per trans. */

	  DMA->TCD[1].NBYTES.MLNO  = DMA_TCD_NBYTES_MLNO_NBYTES(1); /* Transfer 1 byte /minor loop */
	  DMA->TCD[1].SLAST        = DMA_TCD_SLAST_SLAST(-5); 		/* Src addr change -5 bytes after major loop */

	  DMA->TCD[1].DADDR        = DMA_TCD_DADDR_DADDR((uint32_t volatile) &TCD_LC_Dest[6]);	/* Destination Address. */
	  DMA->TCD[1].DOFF         = DMA_TCD_DOFF_DOFF(1);     			/* Dest. adr add 1 byte after transfer */
	  DMA->TCD[1].CITER.ELINKNO= DMA_TCD_CITER_ELINKNO_CITER(5) | 	/* 5 minor loop iterations */
			  	  	  	  	  	 DMA_TCD_CITER_ELINKNO_ELINK(0);   	/* No minor loop chan link */

	  DMA->TCD[1].DLASTSGA     = DMA_TCD_DLASTSGA_DLASTSGA(-5); /* Dest chg -5 bytes after major loop */
	  DMA->TCD[1].CSR          = DMA_TCD_CSR_START(0)       |  /* Clear START status flag */
	                           DMA_TCD_CSR_INTMAJOR(1)    |  /* IRQ after major loop */
	                           DMA_TCD_CSR_INTHALF(0)     |  /* No IRQ after 1/2 major loop */
	                           DMA_TCD_CSR_DREQ(1)        |  /* Disable chan after major loop */
	                           DMA_TCD_CSR_ESG(0)         |  /* Disable Scatter Gather */
	                           DMA_TCD_CSR_MAJORELINK(0)  |  /* No major loop chan link */
	                           DMA_TCD_CSR_ACTIVE(0)      |  /* Clear ACTIVE status flag */
	                           DMA_TCD_CSR_DONE(0)        |  /* Clear DONE status flag */
	                           DMA_TCD_CSR_MAJORLINKCH(0) |  /* Chan # if major loop ch link */
	                           DMA_TCD_CSR_BWC(0);           /* No eDMA stalls after R/W */
	  DMA->TCD[1].BITER.ELINKNO= DMA_TCD_BITER_ELINKNO_BITER(5) |  /* Initial iteration count */
	                           DMA_TCD_BITER_ELINKNO_ELINK(0);    /* No minor loop chan link */
}

/*!
 * DMA  Feature
 * ===================================================
 * Set up DMAMUX to let the DMA receive requests from ADC0 COCO
 */

void DMAMUX_FlexScan_init(void){
	PCC->PCCn[PCC_DMAMUX_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for DMAMUX */
	DMAMUX->CHCFG[0] = DMAMUX_CHCFG_SOURCE(EDMA_REQ_ADC0) | DMAMUX_CHCFG_ENBL_MASK;	/* Set to receive trigger from the ADC to DMACH0 and turn on Enable CH0 */
}


/*!
 * DMA  Feature
 * ===================================================
 * Set up DMA TCD 0 to start filling a string and generate a link to channel 1 after
 * its major loop has ended, set up DMA TCD 1 to finish filling up the string with the
 * remaining amount of bytes.
 *
 */
void DMA_TCD_FlexScan_Config(void){
	  DMA->TCD[0].SADDR        = DMA_TCD_SADDR_SADDR((uint32_t volatile) &(ADC0->R[0])); /* Source Address to ADC0 Result Register */
	  DMA->TCD[0].SOFF         = DMA_TCD_SOFF_SOFF(0);   /* Stay the same source address after Transfers   */
	  DMA->TCD[0].ATTR         = DMA_TCD_ATTR_SMOD(0)  | /* Src. modulo feature not used */
	                             DMA_TCD_ATTR_SSIZE(2) | /* Src. read 32 bits per transfer */
	                             DMA_TCD_ATTR_DMOD(0)  | /* Dest. modulo feature not used */
	                             DMA_TCD_ATTR_DSIZE(2);  /* Dest. write 32 bits per trans. */

	  DMA->TCD[0].NBYTES.MLNO  = DMA_TCD_NBYTES_MLNO_NBYTES(4); /* Transfer 4 byte /minor loop */
	  DMA->TCD[0].SLAST        = DMA_TCD_SLAST_SLAST(0); 		/* No Source Address change after major loop */

	  DMA->TCD[0].DADDR        = DMA_TCD_DADDR_DADDR((uint32_t volatile) &ADC_Results[0]);	/* Destination Address to ADC_Results array. */
	  DMA->TCD[0].DOFF         = DMA_TCD_DOFF_DOFF(4);     			/* Dest. adr add 4 bytes after transfers */
	  DMA->TCD[0].CITER.ELINKYES=DMA_TCD_CITER_ELINKYES_CITER_LE(12)| 	/* 12 minor loop iterations */
	  	  	  	  	  	  	  	 DMA_TCD_CITER_ELINKYES_ELINK_MASK|	/* Enable Linking Channel after Minor Loop */
			  	  	  	  	  	 DMA_TCD_CITER_ELINKYES_LINKCH(1);  /* Link to channel 1 after minor loop */

	  DMA->TCD[0].DLASTSGA     = DMA_TCD_DLASTSGA_DLASTSGA(-48); /* Dest. chg -48 bytes after major loop */
	  DMA->TCD[0].CSR          = DMA_TCD_CSR_START(0)       |  /* Clear START status flag */
	                             DMA_TCD_CSR_INTMAJOR(1)    |  /* IRQ after major loop */
	                             DMA_TCD_CSR_INTHALF(0)     |  /* No IRQ after 1/2 major loop */
	                             DMA_TCD_CSR_DREQ(1)        |  /* Disable chan after major loop */
	                             DMA_TCD_CSR_ESG(0)         |  /* Disable Scatter Gather */
	                             DMA_TCD_CSR_MAJORELINK(1)  |  /* Activate major loop chan link */
	                             DMA_TCD_CSR_ACTIVE(0)      |  /* Clear ACTIVE status flag */
	                             DMA_TCD_CSR_DONE(0)        |  /* Clear DONE status flag */
	                             DMA_TCD_CSR_MAJORLINKCH(1) |  /* Chan # if major loop ch link (Channel 1 in this case) */
	                             DMA_TCD_CSR_BWC(0);           /* No eDMA stalls after R/W */
	  DMA->TCD[0].BITER.ELINKYES= DMA_TCD_BITER_ELINKYES_BITER(12)|  /* Initial iteration count */
	  	  	  	  	  	  	  	 DMA_TCD_BITER_ELINKYES_ELINK_MASK|	/* Enable Linking Channel after Minor Loop */
			  	  	  	  	  	 DMA_TCD_BITER_ELINKYES_LINKCH(1); /* Minor loop Linking Channel */


	  /*Set up DMA TCD CH 1 */

	  DMA->TCD[1].SADDR        = DMA_TCD_SADDR_SADDR((uint32_t volatile) &ADC_SC1A_CH[0]); /* Source Address to Array with ADC channels to read */
	  DMA->TCD[1].SOFF         = DMA_TCD_SOFF_SOFF(4);   /* Src. addr add 4 byte after Transfers   */
	  DMA->TCD[1].ATTR         = DMA_TCD_ATTR_SMOD(0)  | /* Src. modulo feature not used */
			  	  	  	  	  	  DMA_TCD_ATTR_SSIZE(2) | /* Src. read 32 bits per transfer */
								  DMA_TCD_ATTR_DMOD(0)  | /* Dest. modulo feature not used */
								  DMA_TCD_ATTR_DSIZE(2);  /* Dest. write 32 bits per trans. */

	  DMA->TCD[1].NBYTES.MLNO  = DMA_TCD_NBYTES_MLNO_NBYTES(4); /* Transfers 4 byte /minor loop */
	  DMA->TCD[1].SLAST        = DMA_TCD_SLAST_SLAST(-12); 		/* Src addr change -12 bytes after major loop */

	  DMA->TCD[1].DADDR        = DMA_TCD_DADDR_DADDR((uint32_t volatile) &(ADC0->SC1[0]));	/* Destination Address to ADC SC1_A register*/
	  DMA->TCD[1].DOFF         = DMA_TCD_DOFF_DOFF(0);     			/* No destination change after transfer */
	  DMA->TCD[1].CITER.ELINKNO= DMA_TCD_CITER_ELINKNO_CITER(3) | 	/* 3 minor loop iterations */
			  	  	  	  	  	 DMA_TCD_CITER_ELINKNO_ELINK(0);   	/* No minor loop chan link */

	  DMA->TCD[1].DLASTSGA     = DMA_TCD_DLASTSGA_DLASTSGA(0); /* No dest. change after major loop */
	  DMA->TCD[1].CSR          = DMA_TCD_CSR_START(0)       |  /* Clear START status flag */
	                           DMA_TCD_CSR_INTMAJOR(0)    |  /* No IRQ after major loop */
	                           DMA_TCD_CSR_INTHALF(0)     |  /* No IRQ after 1/2 major loop */
	                           DMA_TCD_CSR_DREQ(1)        |  /* Disable chan after major loop */
	                           DMA_TCD_CSR_ESG(0)         |  /* Disable Scatter Gather */
	                           DMA_TCD_CSR_MAJORELINK(0)  |  /* No major loop chan link */
	                           DMA_TCD_CSR_ACTIVE(0)      |  /* Clear ACTIVE status flag */
	                           DMA_TCD_CSR_DONE(0)        |  /* Clear DONE status flag */
	                           DMA_TCD_CSR_MAJORLINKCH(0) |  /* Chan # if major loop ch link */
	                           DMA_TCD_CSR_BWC(0);           /* No eDMA stalls after R/W */
	  DMA->TCD[1].BITER.ELINKNO= DMA_TCD_BITER_ELINKNO_BITER(3) |  /* Initial iteration count */
	                           DMA_TCD_BITER_ELINKNO_ELINK(0);    /* No minor loop chan link */
}


