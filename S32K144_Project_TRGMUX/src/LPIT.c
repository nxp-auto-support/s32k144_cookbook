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

#include "device_registers.h"								/* include peripheral declarations */
#include "LPIT.h"


/*!
* @brief LPIT0 Initialization for Chain Mode.
*
* @param[uint32_t tval_ch1] Timeout Value for Channel 1
* @param[uint32_t tval_ch2] Timeout Value for Channel 2
*/
void LPIT0_chain_init (uint32_t tval_ch1, uint32_t tval_ch2)
{
	/* Module Configuration */
	PCC -> PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    		/* Clock Source = 6 (SPLL2_DIV2_CLK = 40 MHz) */
	PCC -> PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; 		/* Enable CLK for LPIT registers */

	LPIT0 -> MCR |= LPIT_MCR_M_CEN_MASK;  					/* DBG_EN = 0: Timer channels stop in Debug mode */
															/* DOZE_EN = 0: Timer channels are stopped in DOZE mode */
															/* SW_RST = 0: SW reset does not reset timer channels and registers */
															/* M_CEN = 1: Enable module clock (allows writing other LPIT0 registers) */

	/* LPIT0 Channel 2 Configuration */
	LPIT0 -> TMR[2].TCTRL |=  LPIT_TMR_TCTRL_CHAIN_MASK;	/* CHAIN = 1: Channel chaining is enabled */
	LPIT0 -> TMR[2].TVAL = tval_ch2;      					/* Channel 2 Timeout period = 10 counts */
	LPIT0 -> MIER |= LPIT_MIER_TIE2_MASK;  					/* TIE2 = 1: Timer Interrupt Enabled for Channel 2 */
	LPIT0 -> TMR[2].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK; 		/* T_EN = 1: Timer channel is enabled */
															/* MODE = 0: 32 Periodic counter mode */

	/* LPIT0 Channel 1 Configuration */
	LPIT0 -> TMR[1].TVAL = tval_ch1;      					/* Channel 1 Timeout period = 1 second */
															/* Timeout Period = TVAL / CLK = 40000000 / 40 MHz */
	LPIT0 -> MIER |= LPIT_MIER_TIE1_MASK;  					/* TIE1 = 1: Timer Interrupt Enabled for Channel 1 */
	LPIT0 -> TMR[1].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;		/* T_EN = 1: Timer channel is enabled */
															/* MODE = 0: 32 Periodic counter mode */
}


/*!
* @brief LPIT0 Initialization.
*
* @param[uint32_t tval_ch0] Timeout Value for Channel 0
*/
void LPIT0_init (uint32_t tval_ch0)
{
	/* Module Configuration */
	PCC -> PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(2);    		/* Clock Source = 2 (SIRCDIV2_CLK = 8 MHz) */
	PCC -> PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; 		/* Enable CLK for LPIT registers */

	LPIT0 -> MCR |= LPIT_MCR_M_CEN_MASK;  					/* DBG_EN = 0: Timer channels stop in Debug mode */
															/* DOZE_EN = 0: Timer channels are stopped in DOZE mode */
															/* SW_RST = 0: SW reset does not reset timer channels and registers */
															/* M_CEN = 1: Enable module clock (allows writing other LPIT0 registers) */

	/* LPIT0 Channel 0 Configuration */
	LPIT0 -> TMR[0].TVAL = tval_ch0;      					/* Channel 0 Timeout period = 4 seconds */
															/* Timeout Period = TVAL / CLK = 32000000 / 8 MHz */
	LPIT0 -> MIER |= LPIT_MIER_TIE0_MASK;  					/* TIE0 = 1: Timer Interrupt Enabled for Channel 0 */
	LPIT0 -> TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;		/* T_EN = 1: Timer channel is enabled */
}
