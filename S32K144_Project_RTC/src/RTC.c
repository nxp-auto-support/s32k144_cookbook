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
#include "RTC.h"

void RTC_init(void)
{
	/*!
	 * RTC Clocking:
	 * ==========================
	 */
    uint32_t sim_lpoclks_mask = 0;

    /* Select 32kHz LPO clock for RTC_CLK */
    sim_lpoclks_mask = SIM->LPOCLKS|
            		   SIM_LPOCLKS_RTCCLKSEL(0b01) |
					   SIM_LPOCLKS_LPO32KCLKEN_MASK;

    SIM->LPOCLKS = sim_lpoclks_mask;	    /* Write once register */
    PCC->PCCn[PCC_RTC_INDEX] |= PCC_PCCn_CGC_MASK;      /* Enable clock for RTC */

    /*!
     * RTC Initialization:
     * ===========================
     */
    RTC->SR &= ~RTC_SR_TCE_MASK;		/* Disable RTC module */
    RTC->TPR &= 0;	    				/* Time prescaler register is reset to zero */
    RTC->TSR = RTC_INITIAL_SEC_VALUE;	/* Start count from any value different than 0 */

    RTC->TAR = RTC_INITIAL_SEC_VALUE + RTC_ALARM_PERIOD;	/* Set alarm time */

    /*!
     *  Configure RTC
     *  =====================
     */
    RTC->TCR &= 0;  			/* Set compensation to 0 */
    RTC->CR =
            RTC_CR_LPOS(0) |    /* Use RTC_CLK instead of LPO */
            RTC_CR_CPS(1);      /* Select TSIC output for RTC_CLKOUT */

    RTC->IER = RTC_IER_TSIC(0) |	/* Selects 1Hz output */
            RTC_IER_TAIE_MASK;      /* Enable alarm interrupt */

    /* Run RTC */
    RTC->SR |= RTC_SR_TCE_MASK;
}

