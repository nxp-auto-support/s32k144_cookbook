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
#include "lpi2c.h"

void LPI2C_init(void)
{
    PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_PCS(2)        /* Clk src: SIRCDIV2_CLK */
                              | PCC_PCCn_CGC_MASK;   	/* Enable clock for LPI2C0 */


    LPI2C0->MCFGR1 = 	LPI2C_MCFGR1_PRESCALE(2)		/* Prescale = 4*/
					|LPI2C_MCFGR1_IGNACK_MASK;		/* Ignore NACK*/

	/* SCL_freq = Input_freq / (2^PRESCALER * (CLKLO + CLKHI + 2))*/

    LPI2C0->MCCR0 = LPI2C_MCCR0_CLKLO(18)
                | LPI2C_MCCR0_CLKHI(6)
                | LPI2C_MCCR0_SETHOLD(6)
                | LPI2C_MCCR0_DATAVD(3);



    LPI2C0->MFCR = LPI2C_MFCR_TXWATER(0)	/* Transmitter Water mark set to 0*/
                  |LPI2C_MFCR_RXWATER(3);	/* Receiver Water mark set to 3*/

    LPI2C0->MCR |= LPI2C_MCR_MEN_MASK /* Enable LPI2C as master */
            | LPI2C_MCR_DBGEN_MASK;
}


void LPI2C_Transmit (void)
{
	LPI2C0->MTDR = (0x05<<8)|((0x1E<<1)|0);
}


