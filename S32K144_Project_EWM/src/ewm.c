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

#include "ewm.h"
#include "device_registers.h"

void EWM_init(void)
{

	/*!
	 *EWM clocking
	 *==========================================
	 */
	PCC->PCCn[PCC_EWM_INDEX] |= PCC_PCCn_CGC_MASK;
	EWM->CLKPRESCALER = 127;	/*! Prescaled clock frequency =
								 *	low power clock source frequency / ( 1 + CLK_DIV ) =
								 *  LPO 128KHz / 128 = 1KHz.
								 */

	/*!
	 * The EWM compare registers are used to create a
	 * refresh window to refresh the EWM module.
	 */
	EWM->CMPH = 0xFE; 	/* When the counter passes this value outb will be asserted */
	EWM->CMPL = 0x0F;	/* Refresh window is [15ms - 255ms]. */

	EWM->CTRL |= EWM_CTRL_INTEN_MASK| 	/* Enable interrupt */
				 EWM_CTRL_EWMEN_MASK; 	/* Enable EWM */
}

void WDOG_EWM_IRQHandler(void)
{
	/* EWM_OUT_b is asserted */
	EWM->CTRL &= ~EWM_CTRL_INTEN_MASK;	/* de-assert interrupt request */
										/* does not affect EWM_OUT_b. */
}
