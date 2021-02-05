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

#include "pdb.h"

volatile static uint8_t g_errorFlag = 0;

void PDB0_init(void)
{


	/* ----- PDB configuration ----- */
	    PCC->PCCn[PCC_PDB0_INDEX] |= PCC_PCCn_CGC_MASK;     /* Enable PDB clock */

	    PDB0->MOD = PDB_MOD_MOD(14400);      /* Period is set to 1.2 ms */
	    PDB0->IDLY = 10000;                  /* Interrupt is set on 1ms approx (0.833 us * 1200  = 999.6 us) */

	    PDB0->SC = PDB_SC_PDBEIE_MASK |     /* Enable Sequence error interrupt */
	    			PDB_SC_PRESCALER_MASK |   /* PDB frequency is: PDB clock (System Clock) / ((2^PRESCALER) * MULT) */
	                PDB_SC_MULT(0b11) |      /* Selects PDB0_TRG_IN0 input */
	                PDB_SC_LDMOD(0)| /* Load mode 0: The internal registers are loaded with the values from their buffers  immediately after 1 is written to SC[LDOK]. */
					PDB_SC_TRGSEL(0b1111)|/*Software trigger*/
					PDB_SC_CONT(1);       /*Continuous mode*/
	    PDB0->CH[0].C1|=1|PDB_C1_TOS_MASK;
	    PDB0->CH[0].DLY[0] = 1000;

	    PDB0->SC |= PDB_SC_PDBEN_MASK |     /* Enable PDB */
	            PDB_SC_LDOK_MASK;           /* Load values for MOD, IDLY, CHnDLYm, and POyDLY registers */
	    PDB0->SC |= PDB_SC_SWTRIG_MASK
	    		|PDB_SC_LDOK_MASK;

}

