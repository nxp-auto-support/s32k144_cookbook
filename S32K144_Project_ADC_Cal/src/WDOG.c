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
#include "WDOG.h"

/*!
* @brief WDOG Initialization for Window Mode.
*/
void WDOG_init (void)
{
	WDOG -> CNT = 0xD928C520;  											/* Unlock WDOG to reconfigure. See 'Unlocking the Watchdog' in RF */

	while(((WDOG -> CS & WDOG_CS_ULK_MASK) >> WDOG_CS_ULK_SHIFT) == 0); /* Wait until registers are unlocked */

	WDOG -> TOVAL = 500;    											/* Establish WDOG timeout in (5000*256) * (1/128 kHz) = 10s */

	WDOG -> WIN = 250;    												/* Establish the earliest time after the refresh is valid in 2500 * (1/128 kHz) = 5s */


	WDOG -> CS |= WDOG_CS_WIN_MASK										/* [15] WIN = 1 Enable window mode */
			   |  WDOG_CS_CMD32EN_MASK									/* [13] CMD32EN = 1 Enable support for 32-bit refresh/unlock command write words */
			   |  WDOG_CS_PRES_MASK										/* [12] PRES = 1 Enable 256 prescaler */
			   |  WDOG_CS_CLK(0b01)										/* [9-8] CLK = 0b01 Select clock source as LPO = 128 KHz. See 'Peripheral Module Clocking' in RF */
	    	   |  WDOG_CS_EN_MASK										/* [7] EN = 1 Enable WDOG */
	    	   |  WDOG_CS_INT_MASK										/* [6] INT = 1 Enable WDOG interrupts */
			   |  WDOG_CS_UPDATE_MASK;									/* [5] UPDATE = 1 Allow updates */
			   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	/* DBG, WAIT, STOP = 0 WDOG disable in chip debug/wait/stop modes */

	while(((WDOG -> CS & WDOG_CS_RCS_MASK) >> WDOG_CS_RCS_SHIFT) == 0);	/* Wait until new configuration takes effect */

	RCM -> SRIE |= RCM_SRIE_WDOG_MASK									/* Enable WDOG interrupt */
				|  RCM_SRIE_DELAY(0b11);								/* Reset Delay Time: 514 LPO cycles = ~4ms */
}
