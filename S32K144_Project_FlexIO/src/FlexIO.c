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
#include "FlexIO.h"

void FLEXIO_UART_init(void)
{
	/*!
	 * Emulating UART by using FlexIO module
	 * =======================================
	 */

    PCC->PCCn[PCC_FlexIO_INDEX] &= ~PCC_PCCn_CGC_MASK;	/* Disable FLEXIO clock for config.	*/
    PCC->PCCn[PCC_FlexIO_INDEX] |= PCC_PCCn_PCS(2)		/* Select clk option 2 = SIRCDIV2_CLK	*/
                                |  PCC_PCCn_CGC_MASK;	/* Enable FLEXIO clock 				*/

    /*!
     * FlexIO Timer Configuration:
     * =============================
     */
        FLEXIO->TIMCFG[0] |=
                FLEXIO_TIMCFG_TIMRST(6)		/* Timer reset on trigger rising edge */
                |FLEXIO_TIMCFG_TIMOUT(0)	/* Output logic 1 when enabled */
                |FLEXIO_TIMCFG_TIMDIS(2)	/* Timer disabled on Timer compare */
                |FLEXIO_TIMCFG_TIMENA(2)    /* Timer enabled on Trigger high */
                |FLEXIO_TIMCFG_TSTART(1)	/* Start bit enabled */
                |FLEXIO_TIMCFG_TSTOP(3);	/* Stop bit is enabled on timer compare and timer disable */

     /*!
      * FlexIO Timer Control:
      * ============================
      */
        FLEXIO->TIMCTL[0] |=
                FLEXIO_TIMCTL_TRGSEL(1)		/* Trigger select: Shifter 0 status flag */
                |FLEXIO_TIMCTL_TRGPOL(1)	/* Trigger active low */
                |FLEXIO_TIMCTL_TIMOD(1)		/* Dual 8-bit counters baud mode */
                |FLEXIO_TIMCTL_TRGSRC(1)	/* Internal trigger */
                |FLEXIO_TIMCTL_PINSEL(2)	/* Select FXIO_D2 */
                |FLEXIO_TIMCTL_PINCFG(0); 	/* timer pin output disabled */

        /*!
         * FlexIO Timer Compare Value:
         * =============================
         * In 8-bit baud counter mode, the lower 8-bits configure the baud rate
         * divider equal to (CMP[7:0] + 1) * 2. The upper 8-bits configure the
         * number of bits in each word equal to (CMP[15:8] + 1) / 2.
         */
        FLEXIO->TIMCMP[0] = 0x0FCF;	/* 8 bits Transfer */
        							/* Baud Rate = 19200b/s */

        /*!
         * FlexIO Shifter Configuration:
         * ===============================
         */
        FLEXIO->SHIFTCFG[0] |=
                FLEXIO_SHIFTCFG_SSTART(2)	/* Start bit '0' */
                |FLEXIO_SHIFTCFG_SSTOP(3);	/* Stop bit '1'  */

        /*!
         * FlexIO Shifter Control:
         * =============================
         */
        FLEXIO->SHIFTCTL[0] |=
                FLEXIO_SHIFTCTL_TIMSEL(0)	/* Select Timer 0 */
                |FLEXIO_SHIFTCTL_PINSEL(1)	/* Select FXIO_D1 */
                |FLEXIO_SHIFTCTL_PINCFG(3)	/* Shifter pin as Output */
                |FLEXIO_SHIFTCTL_SMOD(2);	/* Transmit mode */
}

void FLEXIO_UART_transmit_char(char send)
{
	/*!
	 * Transmit a single char:
	 * =======================
	 */
	while(! (FLEXIO->SHIFTSTAT & 0x1) );	/* Wait for buffer empty */
	FLEXIO->SHIFTBUF[0] = send;				/* Send data */
}

void FLEXIO_UART_transmit_string(char data[])
{
	/*!
	 * Transmit whole string:
	 * =========================
	 */
	uint32_t idx=0;
	while(data[idx] != '\0')
	{
		/* Send chars one at a time */
		FLEXIO_UART_transmit_char(data[idx]);
		idx++;
	}
}

void FLEXIO_100KHz_PWM_init(int duty)
{
uint16_t low = ( ( ( (8000000/100000) * (100-duty)) / 100) - 1) << 8;
uint16_t high= ( ( (8000000/100000) * duty) / 100) - 1;

	/*!
	 * Using FlexIO to output a PWM signal.
	 * ======================================
	 */

    /*!
     * FlexIO Timer Configuration:
     * =============================
     */
        FLEXIO->TIMCFG[1] |= 0x00000000;	/* (Defaults) */
                							/* Never reset */
                							/* Output logic 1 when enabled */
                							/* Never disabled */
               								/* Always enabled */
              								/* Start bit disabled */
                							/* Stop bit disabled */
     /*!
      * FlexIO Timer Control:
      * ============================
      */
        FLEXIO->TIMCTL[1] |=
                 FLEXIO_TIMCTL_TIMOD(2)		/* Dual 8-bit counters PWM high mode */
                |FLEXIO_TIMCTL_PINSEL(0)	/* Select FXIO_D0 */
                |FLEXIO_TIMCTL_PINCFG(3); 	/* Timer pin output */

        /*!
         * FlexIO Timer Compare Value:
         * =============================
         * In 8-bit PWM high mode, the lower 8-bits configure the high period
         * of the output to (CMP[7:0] + 1) and the upper 8-bits configure the
         * low period of the output to (CMP[15:8] + 1).
         */
        FLEXIO->TIMCMP[1] = high | low;
        FLEXIO->CTRL = FLEXIO_CTRL_FLEXEN_MASK;	/* Enable FlexIO module */
}
