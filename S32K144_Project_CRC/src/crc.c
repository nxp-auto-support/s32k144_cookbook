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

#include "crc.h"
#include "device_registers.h"

#define POLY (0x04C11DB7)

uint32_t CRC_32bits_calculate(uint8_t *data, uint32_t size)
{
	/*! CRC-32 Algorithm: \n
	 * polynomial = 0x04C11DB7 \n
	 * seed = 0xFFFFFFFF \n
	 * Bits in a byte are transposed for writes. \n
	 * Both bits in bytes and bytes are transposed for read. \n
	 * XOR on reading. \n
	*/

	PCC->PCCn[PCC_CRC_INDEX] |= PCC_PCCn_CGC_MASK;	/* enable CRC clock */

	CRC->CTRL |= CRC_CTRL_TCRC(1);	/* enable 32-bit CRC protocol */
	CRC->CTRL |= CRC_CTRL_TOT(1);	/* Bits in a byte are transposed, while bytes are not transposed. */
	CRC->CTRL |= CRC_CTRL_TOTR(2);	/* Both bits in bytes and bytes are transposed. */
	CRC->CTRL |= CRC_CTRL_FXOR(1);	/* XOR on reading. */

		CRC->GPOLY = POLY;	

	CRC->CTRL |= CRC_CTRL_WAS_MASK;	/* Set CRC_CTRL[WAS] to program the seed value. */
	CRC->DATAu.DATA = 0xFFFFFFFF;	/* seed value */

	CRC->CTRL &= ~CRC_CTRL_WAS_MASK;	/* Clear CRC_CTRL[WAS] to start writing data values. */
	for(;size--;)
	CRC->DATAu.DATA = *data++;	/* write data values */

return (uint32_t) CRC->DATAu.DATA;
}

