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

/*!
 * Description:
 * ==============================================================================================================
 * This example shows how to chain the LPIT0 channels to make a larger timer.
 * LPIT0 module contains 4 channels (0 - 3). Each channel could be used as an independent 32 bit timer.
 * LPIT0 channel n timeout value is stored in TVAL[n] register. This module counts backwards (from TVAL to 0),
 * so when the counter reaches 0, the timeout flag is asserted.
 *
 * If a larger timer is needed Channel 1 and Channel 2 can be chained together to get a 64 bit timer
 * or Channel 1, Channel 2 and Channel 3 can be chained together to get a 96 bit timer
 * Channel 0 can not be chained.
 *
 * In chain mode, the channel n decreases its counter every time channel n-1 timeouts flag is asserted.
 *
 * For this example, only Channel 1 and Channel 2 are being chained.
 * A LED will blink 5 times with a 1 second period. To count the blinking period, the
 * Channel 1 is used. To count how many times the LED has blinked Channel 2 is used.
 *
 * Channel 1 is set with a TVAL of 1s and Channel 2 is set with a TVAL of 10 counts.
 * In other words every 1s, the Ch1 timeout flag will be asserted and the Ch2 counting will decrease by 1,
 * meaning that after Ch1 timeout flag is asserted 10 times, the Ch2 timeout flag would be asserted as well.
 *
 * Remember 10 counts = 5 Blinks. The Ch1 toggles the LED from On to Off. This is
 * equal to 1 second on and 1 second off, repeating the cycle 10 times.
 *
 * Every time a LED turns on or off, Ch1 timeout flag is asserted. Every time the LED changes
 * its color, Ch2 timeout flag is asserted.
 *
 * */

#include "device_registers.h" 				/* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
#include "LPIT.h"

#define PTD0 (0)
#define PTD15 (15)
#define PTD16 (16)


uint32_t LEDs[] = {PTD0, PTD15, PTD16};					/* LEDs array */
uint8_t LEDs_index = 0;									/* Variable for the array position */

uint32_t LPIT0_ch1_flag_counter = 0;					/* Counter for the timeout channel 1 flag */
uint32_t LPIT0_ch2_flag_counter = 0;					/* Counter for the timeout channel 1 flag */

/*!
* @brief PORTn Initialization
*/
void PORT_init (void)
{
	/*!
	*           Pins Definitions
	* =====================================
	*
	*    Pin Number     |    Function
	* ----------------- |------------------
	* PTD0				| GPIO [BLUE_LED]
	* PTD15             | GPIO [RED_LED]
	* PTD16			 	| GPIO [GREEN_LED]
	*/

	PCC -> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; 		/* Enable clock for PORT D */

	PORTD -> PCR[PTD0] = PORT_PCR_MUX(1);  					/* Port D0: MUX = GPIO */
	PORTD -> PCR[PTD15] = PORT_PCR_MUX(1); 					/* Port D15: MUX = GPIO */
	PORTD -> PCR[PTD16] = PORT_PCR_MUX(1);  				/* Port D16: MUX = GPIO */

	PTD -> PDDR |= 1 << PTD0								/* Port D0: Data Direction = output */
				|  1 << PTD15	    						/* Port D15: Data Direction = output */
	  	  	  	|  1 << PTD16;    							/* Port D16: Data Direction = output */

	PTD -> PSOR |= 1 << PTD0								/* Turn-Off all LEDs */
				|  1 << PTD15
			    |  1 << PTD16;
}

void WDOG_disable (void)
{
	WDOG -> CNT = 0xD928C520;     			/* Unlock watchdog */
	WDOG -> TOVAL = 0x0000FFFF;   			/* Maximum timeout value */
	WDOG -> CS = 0x00002100;    			/* Disable watchdog */
}

void Enable_Interrupt (uint8_t vector_number)
{
	S32_NVIC->ISER[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
	S32_NVIC->ICPR[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

int main (void)
{
	/*!
	 * Initialization:
	 */
	WDOG_disable();							/* Disable watchdog */
	SOSC_init_8MHz();      					/* Initialize system oscilator for 8 MHz xtal */
	SPLL_init_160MHz();    					/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); 					/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();           					/* Configure ports */

	LPIT0_chain_init(40000000, 10);			/* Channel 1 Timeout period = 1 second */
											/* Timeout Period = TVAL / CLK = 40000000 / 40 MHz */
											/* Channel 2 Timeout period = 10 counts */

	Enable_Interrupt(LPIT0_Ch2_IRQn);		/* Enable LPIT0 Channel 2 interrupt vector */
	Enable_Interrupt(LPIT0_Ch1_IRQn);		/* Enable LPIT0 Channel 1 interrupt vector */

	/* Since LPIT0_Ch2 and LPIT_Ch1 could generate an interrupt at the same time, is important to
	 * set a priority. In this case LPIT_Ch2 has a 0 priority level (default) and LPIT_Ch1 has a 2
	 * priority level. Remember, 0 priority level is the highest priority.
	 */
	S32_NVIC -> IP[LPIT0_Ch1_IRQn] = 0x20;	/* Level of priority (0 - 15) */

	/*!
	* Infinite for:
	*/
	for(;;)
	{
	}

	return 0;
}


void LPIT0_Ch1_IRQHandler (void)
{
	/* Check the Timeout Interrupt Flag for Channel 1 */
	if ((LPIT0 -> MSR & LPIT_MSR_TIF1_MASK) == LPIT_MSR_TIF1_MASK)
	{
		LPIT0_ch1_flag_counter++;         		/* Increment LPIT0 Ch1 timeout counter */

		LPIT0 -> MSR |= LPIT_MSR_TIF1_MASK; 	/* Clear LPIT0 Timeout Flag Channel 1 (W1C) */

		PTD -> PTOR |= 1 << LEDs[LEDs_index];	/* Toggle the corresponding LED */
	}
}

void LPIT0_Ch2_IRQHandler (void)
{
	/* Check the Timeout Interrupt Flag for Channel 2 */
	if ((LPIT0 -> MSR & LPIT_MSR_TIF2_MASK) == LPIT_MSR_TIF2_MASK)
	{
		LPIT0_ch2_flag_counter++;         		/* Increment LPIT0 Ch2 timeout counter */

		LPIT0 -> MSR |= LPIT_MSR_TIF2_MASK; 	/* Clear LPIT0 Timeout Flag Channel 1 (W1C) */

		PTD -> PSOR |= 1 << LEDs[LEDs_index];	/* Turn Off the current toggled LED */

		if (LEDs_index < 2) LEDs_index++;		/* Increment LED index */

		else LEDs_index = 0;					/* Reset LED index */
	}
}
