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

#include "acmp.h"
#include "device_registers.h"

void ACMP_Init(unsigned char vosel)
{
	/*!
	 *CMP clocking
	 *==========================================
	 */
    PCC->PCCn[PCC_CMP0_INDEX] |= PCC_PCCn_CGC_MASK;     /* Enable bus clock in CMP */

  /*!
	 *CMP initialization
	 *==========================================
	 */
    CMP0->C1 =   CMP_C1_INPSEL(0b00)    /* Positive terminal selection from the 8-bit DAC output */
               | CMP_C1_INNSEL(0b01)    /* Negative selection from external analog mux */

               | CMP_C1_VRSEL(0b0)      /* Vin1 (VDDA) as supply reference Vin */

               | CMP_C1_MSEL(0)         /* PSEL Reference Input (0) */

               | CMP_C1_VOSEL(vosel)    /* DAC output voltage (Ref.) */
               | CMP_C1_DACEN_MASK;     /* DAC enable */

    CMP0->C0 |= CMP_C0_IEF_MASK			/* Comparator Interrupt Enable Falling */
    		 |  CMP_C0_IER_MASK;		/* Comparator Interrupt Enable Rising */

    CMP0->C0 |= CMP_C0_COS_MASK			/* COUTA: unfiltered comparator output */
    		 |  CMP_C0_INVT_MASK		/* Inverts the comparator output */
    		 |  CMP_C0_OPE_MASK			/* Output Pin Enable */
			 |  CMP_C0_EN_MASK;         /* Enable CMP module */
}

