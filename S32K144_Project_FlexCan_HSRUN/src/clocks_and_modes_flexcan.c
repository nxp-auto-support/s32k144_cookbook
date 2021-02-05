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

#include "register_bit_fields.h"								/* include peripheral declarations S32K144 */
#include "clocks_and_modes_flexcan.h"
#include "stdint.h"

void SOSC_init_8MHz (void)
{
    /* System Oscillator (SOSC) initialization for 8 MHz external crystal */
    SCG -> SCG_SOSCCSR_b.LK       = SCG_SOSCCSR_LK_0;         	/* Ensure the register is unlocked */
    SCG -> SCG_SOSCCSR_b.SOSCEN   = SCG_SOSCCSR_SOSCEN_0;     	/* Disable SOSC for setup */
    SCG -> SCG_SOSCCFG_b.EREFS    = SCG_SOSCCFG_EREFS_1;     	/* Setup external crystal for SOSC reference */
    SCG -> SCG_SOSCCFG_b.RANGE    = SCG_SOSCCFG_RANGE_10;     	/* Select 8 MHz range */
    SCG -> SCG_SOSCCSR_b.SOSCEN   = SCG_SOSCCSR_SOSCEN_1;     	/* Enable SOSC reference */
    SCG -> SCG_SOSCDIV_b.SOSCDIV2 = SCG_SOSCDIV_SOSCDIV2_001; 	/* Asynchronous source for FlexCAN */
    SCG -> SCG_SOSCCSR_b.LK       = SCG_SOSCCSR_LK_1;         	/* Lock the register from accidental writes */

    /* Poll for valid SOSC reference, needs 4096 cycles */
    while(!(SCG -> SCG_SOSCCSR_b.SOSCVLD));
}

void SPLL_init_112MHz (void)
{
    /* System PLL (SPLL) initialization for to 112 MHz reference */
    SCG -> SCG_SPLLCSR_b.LK       = SCG_SPLLCSR_LK_0;         	/* Ensure the register is unlocked */
    SCG -> SCG_SPLLCSR_b.SPLLEN   = SCG_SPLLCSR_SPLLEN_0;     	/* Disable PLL for setup */
    SCG -> SCG_SPLLCFG_b.MULT     = 12;                       	/* Select multiply factor of 28 for 112 MHz SPLL_CLK */
    SCG -> SCG_SPLLDIV_b.SPLLDIV2 = SCG_SPLLDIV_SPLLDIV2_001; 	/* Divide by 1 for 112 MHz at SPLLDIV2 output for LPIT */
    SCG -> SCG_SPLLCSR_b.SPLLEN   = SCG_SPLLCSR_SPLLEN_1;     	/* Enable PLL */
    SCG -> SCG_SPLLCSR_b.LK       = SCG_SPLLCSR_LK_1;         	/* Lock register */

    /* Poll for valid SPLL reference */
    while(!(SCG -> SCG_SPLLCSR_b.SPLLVLD));
}

void High_Speed_RUN_init (void)
{
    /* High Speed RUN configuration for output clocks, this register requires 32-bit writes */
    SCG -> SCG_HCCR = SCG_HCCR_SCS_0110 | SCG_HCCR_DIVCORE_0000 | SCG_HCCR_DIVBUS_0001 | SCG_HCCR_DIVSLOW_0011;
}
