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
 * ========================================================================================================
 * The TRGMUX provides an extremely flexible mechanism for connecting various trigger sources
 * to multiple pins/peripherals.
 *
 * In this example LPUART1, CMP0 and LPIT0 Ch0 are connected to each other using the TRGMUX module.
 * However, notice that many more options are available in the S32K1xx_Trigger_Muxing.xlsx attached file
 * in the Reference Manual, so it is possible to set different triggers for different modules.
 *
 * The basic steps to set a trigger source for a module are:
 * 		- Select the trigger source for the desired module in the TRGMUX register
 * 		- Select/enable the trigger input for the desired module (registers are different according to the module)
 *
 * For this project, the trigger sources are set as follows:
 *
 * 		LPIT0_CH0 ---- (triggers) ---> CMP0 ---- (triggers) ----> LPUART1
 *
 * Notice that the order is important, a different order or different modules may require different configurations.
 * Trigger the LPUART1 with the CMP0 is not the same as trigger the CMP0 with the LPUART1.
 *
 * In other words, the LPIT0_CH0 is set with a value of 4s. After this time, the timeout flag is asserted.
 *
 * Until the timeout flag is asserted, not before, CMP0 module performs a sample reading from
 * the CMP0_IN0 pin (PTA0) and compares the input voltage with a desired threshold (2.5 V).
 * The result from the comparison is visible in the CMP0_OUT pin (PTE3). This means that every
 * 4s a CMP0 sample is triggered by the LPIT0_CH0
 *
 * 		If CMP0_IN0 > 2.5 V      CMP0_OUT = 1     Green LED on
 *		If CMP0_IN0 < 2.5 V      CMP0_OUT = 0     Red LED on
 *
 * Also, when the CMP0_OUT = 1, the LPUART1 sends a message. If CMP0_OUT = 0, the message will not be sent.
 * This means that the LPUART1 is triggered by the CMP0_OUT.
 *
 * Remember that is not enough to select the trigger source in the TRIGMUX register, additional configurations
 * may be done. For example:
 *
 * 		CMP0: Enable the Window Mode sampling. Allows to be triggered by a timer.
 * 		      CMP0 -> C0 =  CMP_C0_WE_MASK
 * 		      LPIT0_CH0 is selected as trigger source in the TRGMUX register
 *
 *   	LPUART1: Select to modulate the TXD pin output with an input trigger
 * 		         LPUART1 -> PINCFG = LPUART_PINCFG_TRGSEL(0b11);
 * 		         CMP0_OUT is selected as trigger source in the TRGMUX register
 *
 * NOTE: To change the input voltage for the CMP0 sampling, connect with a jumper or cable, the
 *       CMP0_IN0 pin (PTA0) with the potentiometer output (PTC14) of the EVB
 *       To see the LPUART1 messages, TeraTerm or other software could be used.
 * */

#include "device_registers.h" 							/* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
#include "LPIT.h"
#include "acmp.h"
#include "LPUART.h"

#define PTA0  (0)
#define PTE3  (3)
#define PTC6  (6)
#define PTC7  (7)
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
	* PTA0				| CMP0  [IN0]
	* PTE3              | CMP0  [OUT]
	* PTC6              | UART1 [Rx]
	* PTC7			 	| UART1 [Tx]
	* PTD15             | GPIO  [RED_LED]
	* PTD16			 	| GPIO  [GREEN_LED]
	*/

	PCC -> PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT A */
	PCC -> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT C */
	PCC -> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT D */
	PCC -> PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK; 	/* Enable clock for PORT E */

	PORTA -> PCR[PTA0] = PORT_PCR_MUX(0);				/* Port A0: MUX = Comparator Input Signal */
	PORTE -> PCR[PTE3] = PORT_PCR_MUX(7);				/* Port E3: MUX = Comparator Output Trigger */

	PORTC -> PCR[PTC6] |= PORT_PCR_MUX(2);				/* Port C6: MUX = UART1 RX */
	PORTC -> PCR[PTC7] |= PORT_PCR_MUX(2);   			/* Port C7: MUX = UART1 TX */

	PORTD -> PCR[PTD15] = PORT_PCR_MUX(1); 				/* Port D15: MUX = GPIO */
	PORTD -> PCR[PTD16] = PORT_PCR_MUX(1);  			/* Port D16: MUX = GPIO */

	PTD -> PDDR |= 1 << PTD15	    					/* Port D15: Data Direction = output */
	  	  	  	|  1 << PTD16;    						/* Port D16: Data Direction = output */

	PTD -> PSOR |= 1 << PTD15							/* Turn-Off all LEDs */
			    |  1 << PTD16;
}

void WDOG_disable (void)
{
	WDOG -> CNT = 0xD928C520;     						/* Unlock watchdog */
	WDOG -> TOVAL = 0x0000FFFF;   						/* Maximum timeout value */
	WDOG -> CS = 0x00002100;    						/* Disable watchdog */
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
	WDOG_disable();										/* Disable watchdog */
	SOSC_init_8MHz();      								/* Initialize system oscillator for 8 MHz xtal */
	SPLL_init_160MHz();    								/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); 								/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();           								/* Configure ports */

	/* LPUART1 Initialization at 9600 baud */
	LPUART1_init ( );

	/* Select CMP0_OUT (14) as trigger source for the LPUART1_TX */
	/* Refer to the S32K1xx_Trigger_Muxing.xlsx attached in the Reference Manual */
	TRGMUX -> TRGMUXn[TRGMUX_LPUART1_INDEX] |= TRGMUX_TRGMUXn_SEL0(14);
	TRGMUX -> TRGMUXn[TRGMUX_LPUART1_INDEX] |= TRGMUX_TRGMUXn_LK_MASK;

	/* CMP0 Initialization */
	ACMP_init (127);									/* Threshold reference voltage = 2.5V */
	   	   	   	   	   									/* DACO = (Vin / 256) * (vosel + 1) */
	   	   	   	   	   									/* DACO = (5 / 256) * (127 + 1) */

	/* Select LPIT0_CH0 (17) as trigger source for the CMP0_SAMPLE */
	/* Refer to the S32K1xx_Trigger_Muxing.xlsx attached in the Reference Manual */
	TRGMUX -> TRGMUXn[TRGMUX_CMP0_INDEX] |= TRGMUX_TRGMUXn_SEL0(17);
	TRGMUX -> TRGMUXn[TRGMUX_CMP0_INDEX] |= TRGMUX_TRGMUXn_LK_MASK;

	/* LPIT0 Initialization */
	LPIT0_init(32000000);								/* Channel 0 Timeout period = 4 seconds */
														/* Timeout Period = TVAL / CLK = 32000000 / 8 MHz */

	/* Enable Interrupts */
	Enable_Interrupt(CMP0_IRQn);						/* Enable CMP0 interrupt vector */
	Enable_Interrupt(LPIT0_Ch0_IRQn);					/* Enable LPIT0 Channel 0 interrupt vector */

	/*!
	* Infinite for:
	*/
	for(;;)
	{
		LPUART1_transmit_string("LPUART1_Tx triggered by CMP0_OUT\r\n");
	}

	return 0;
}

void LPIT0_Ch0_IRQHandler (void)
{
	/* Check the Timeout Interrupt Flag for Channel 0 */
	if ((LPIT0 -> MSR & LPIT_MSR_TIF0_MASK) == LPIT_MSR_TIF0_MASK)
	{
		LPIT0 -> MSR |= LPIT_MSR_TIF0_MASK; 			/* Clear LPIT0 Timeout Flag Channel 0 (W1C) */
	}
}


void CMP0_IRQHandler (void)
{
	/* A rising edge on COUT has occurred. Voltage PTA0 > DACO */
	if ((CMP0 -> C0 & CMP_C0_CFR_MASK) == CMP_C0_CFR_MASK)
	{
		PTD -> PSOR |= 1 << PTD15;						/* Turn-Off Red LED */
		PTD -> PCOR |= 1 << PTD16;						/* Turn-On Green LED */

		CMP0 -> C0 |= CMP_C0_CFR_MASK;					/* Clear Analog Comparator Flag Raising (W1C) */
	}

	/* A falling edge on COUT has occurred. Voltage PTA0 < DACO */
	else if ((CMP0 -> C0 & CMP_C0_CFF_MASK) == CMP_C0_CFF_MASK)
	{
		PTD -> PSOR |= 1 << PTD16;						/* Turn-Off Green LED */
		PTD -> PCOR |= 1 << PTD15;						/* Turn-On Red LED */

		CMP0 -> C0 |= CMP_C0_CFF_MASK;					/* Clear Analog Comparator Flag Falling (W1C) */
	}
}
