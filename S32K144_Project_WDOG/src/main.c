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
 * ==========================================================================================
 * This example shows how the WDOG works in Window Mode.
 * A button refreshes the WDOG manually.
 * If the button is pushed too early or too late, the WDOG resets the MCU.
 * Once the WDOG is refreshed in the allowed time, the WDOG counter (CNT) starts again.
 * You must refresh the WDOG periodically to avoid the MCU reset.
 * For this example, a valid refresh is between 5 and 10 seconds from when the WDOG counter started running.
 * This values can be changed in the WIN and TOVAL registers respectively.
 * The UART is only used to display the WDOG status in the terminal at 115200 baud: TeraTerm or other software.
 * */

#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
#include "LPUART.h"
#include "WDOG.h"

#define PTC6 (6)
#define PTC7 (7)
#define PTC12 (12)
#define PTD0 (0)
#define PTD15 (15)
#define PTD16 (16)

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
	* PTC6              | UART1 [Rx]
	* PTC7			 	| UART1 [Tx]
	* PTC12             | GPIO [BUTTON]
	* PTD0				| GPIO [BLUE_LED]
	* PTD15             | GPIO [RED_LED]
	* PTD16			 	| GPIO [GREEN_LED]
	*/

	PCC -> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; 					/* Enable clock for PORT C */
	PCC -> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; 					/* Enable clock for PORT D */

	PORTD -> PCR[PTD0] = PORT_PCR_MUX(1);  								/* Port D0: MUX = GPIO */
	PORTD -> PCR[PTD15] = PORT_PCR_MUX(1); 								/* Port D15: MUX = GPIO */
	PORTD -> PCR[PTD16] = PORT_PCR_MUX(1);  							/* Port D16: MUX = GPIO */

	PTD -> PDDR |= 1<<PTD0												/* Port D15: Data Direction = output */
				|  1<<PTD15	    										/* Port D15: Data Direction = output */
	  	  	  	|  1<<PTD16;    										/* Port D16: Data Direction = output */

	PTD -> PSOR |= 1<<PTD0												/* Turn-Off all LEDs */
				|  1<<PTD15
			    |  1<<PTD16;

	PORTC -> PCR[PTC12] = PORT_PCR_MUX(1)  								/* Port C12: MUX = GPIO */
					    | PORT_PCR_IRQC(0b1011);						/* ISF flag and interrupt on either edge. */

	PTC -> PDDR &= ~(1<<PTC12);											/* Port C12: Data Direction = input */

	PORTC -> PCR[PTC6] |= PORT_PCR_MUX(2);								/* Port C6: MUX = UART1 RX */
	PORTC -> PCR[PTC7] |= PORT_PCR_MUX(2);   							/* Port C7: MUX = UART1 TX */
}

void Enable_Interrupt(uint8_t vector_number)
{
	S32_NVIC->ISER[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
	S32_NVIC->ICPR[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

int main(void)
{
	/*!
	 * Initialization:
	 */
	SOSC_init_8MHz();      				/* Initialize system oscilator for 8 MHz xtal */
	SPLL_init_160MHz();    				/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); 				/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();           				/* Configure ports */
	Enable_Interrupt(PORTC_IRQn);		/* Enable PORTC interrupt vector */

	WDOG_init();
	Enable_Interrupt(WDOG_EWM_IRQn);	/* Enable WDOG interrupt vector */
	LPUART1_init();
	LPUART1_transmit_string("\n\n\r============================================");
	LPUART1_transmit_string("\n\rWDOG counter is running. Try to refresh it!\n\n\r");

	/*!
	* Infinite for:
	*/
	  for(;;)
	  {
	  }
}

void PORTC_IRQHandler (void)
{
	if((PTC -> PDIR & (1<<PTC12)) == (1<<PTC12))						/* The button is pressed */
	{
		PORTC -> PCR[PTC12] |= PORT_PCR_ISF_MASK;						/* Clear Interrupt Status Flag */

		/* If the refresh happens between the allowed time */
		if(((WDOG -> CNT) < (WDOG -> TOVAL)) && ((WDOG -> CNT) > (WDOG -> WIN)))
		{
			PTD -> PSOR |= 1<<PTD15;									/* Turn-Off RED_LED */
			PTD -> PCOR |= 1<<PTD16;									/* Turn-On GREEN_LED */
			LPUART1_transmit_string("\rWDOG refresh was successful! Remember to refresh it periodically!\n\n\r");
		}

		/* If the refresh happens before the allowed time */
		if((WDOG -> CNT) < (WDOG -> WIN)){
			PTD -> PSOR |= 1<<PTD16;									/* Turn-Off GREEN_LED */
			PTD -> PCOR |= 1<<PTD15;									/* Turn-On RED_LED */
			LPUART1_transmit_string("\rWDOG refresh was too early! Try again!");
			S32_NVIC -> ICER[WDOG_EWM_IRQn/32] |= (1<<(WDOG_EWM_IRQn%32));

		}

		RCM -> SRIE &= ~RCM_SRIE_GIE_MASK;								/* Disable global interrupts */
		WDOG -> CNT  = 0xB480A602; 										/* Refresh WDOG. See "Refreshing the Watchdog" in RF */
		RCM -> SRIE |= RCM_SRIE_GIE_MASK;								/* Enable global interrupts */
	}
}

void WDOG_EWM_IRQHandler (void)
{
    /* WDOG interrupt flag active */
	if((WDOG -> CS & WDOG_CS_FLG_MASK) == WDOG_CS_FLG_MASK)
	{
    	WDOG -> CS |= WDOG_CS_FLG_MASK;      							/* Clear the flag */
        PTD -> PSOR |= 1<<PTD16;										/* Turn-Off GREEN_LED */
        PTD -> PCOR |= 1<<PTD15;										/* Turn-On RED_LED */
        LPUART1_transmit_string("TIMEOUT! WDOG refresh never came!");
    }
}
