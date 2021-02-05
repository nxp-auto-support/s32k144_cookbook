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
 * This code interactively shows how the internal calibration of the ADC module
 * affects the reading of the result register (R).
 * The ADC calibration is made by the module itself, however if the internal calibration is not enough,
 * the user can modify two registers for the Gain (UG) and the Offset (USR_OFS) of the result.
 *
 * Instructions:
 * 		Valid User Gain values are between 0 - 1023.
 *  	Valid User Offset values are between 0 - 255.
 *  	Offset and Gain values are in 2's-complement format.
 *  	There are negative and positive values. MSB determines the sign.
 *  	Press ENTER to send the Gain and Offset value.
 *
 * The UART is only used to display the ADC Result status in the terminal at 9600 baud: TeraTerm or other software.
 *
 * To perform a good calibration, the module must be calibrated ONCE along the project. If you calibrate the module more than once, 
 * the ADC readings will become imprecise.
 *
 * However, for this project and only to show the differences between ADC readings with different calibration parameters,
 * the calibration can be done several times because after each ADC reading the WDOG resets the MCU. 
 * */

#include "device_registers.h"           /* include peripheral declarations */
#include "clocks_and_modes.h"
#include "ADC.h"
#include "LPUART.h"
#include "WDOG.h"

#define PTC6 (6)
#define PTC7 (7)

uint16_t answer = 0;
uint16_t gain = 0;
uint16_t offset = 0;
uint32_t adc_mV_result = 0;
uint8_t state = 0;

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
	*/

	PCC -> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; 			/* Enable clock for PORT C */

	PORTC -> PCR[PTC6] |= PORT_PCR_MUX(2);						/* Port C6: MUX = UART1 RX */
	PORTC -> PCR[PTC7] |= PORT_PCR_MUX(2);   					/* Port C7: MUX = UART1 TX */
}

void Enable_Interrupt(uint8_t vector_number)
{
	S32_NVIC->ISER[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
	S32_NVIC->ICPR[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

int main (void)
{
	/*!
	* Initialization:
	*/
	SOSC_init_8MHz();      					/* Initialize system oscillator for 8 MHz xtal */
	SPLL_init_160MHz();    					/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); 					/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();		    				/* Configure ports */
	LPUART1_init();							/* LPUART1 initialization */

	/* Welcome message */
	LPUART1_transmit_string("\r\n=============================================================================\r\n");
	LPUART1_transmit_string("This code interactively shows how the internal calibration of the ADC module\r\n");
	LPUART1_transmit_string("affects the reading of the result register (R). In addition to the internal\r\n");
	LPUART1_transmit_string("calibration, there is the possibility of modifying two registers for the Gain\r\n");
	LPUART1_transmit_string("(UG) and the Offset (USR_OFS) of the result by the user.\r\n\r\n");

	/* Instructions */
	LPUART1_transmit_string("Instructions:\r\n");
	LPUART1_transmit_string("	- Valid User Gain values are between 0 - 1023.\r\n");
	LPUART1_transmit_string("	- Valid User Offset values are between 0 - 255.\r\n");
	LPUART1_transmit_string("	- Offset and Gain values are in 2's-complement format.\r\n");
	LPUART1_transmit_string("	- There are negative and positive values. MSB determines the sign.\r\n");
	LPUART1_transmit_string("	- Press ENTER to send the Gain and Offset value. \r\n\r\n");

	/* Ask for initial calibration */
	LPUART1_transmit_string("Would you like to calibrate the ADC module? y/n.\r\n\r\n");
	LPUART1_transmit_string("> ");

	for(;;)
	{
		answer = LPUART1_receive_char();									/* Receive answer from the question above */

		/* ADC module with calibration */
		if(answer == 'y')
		{
			state = 1;
			LPUART1_transmit_string("\r\n\r\n");
			LPUART1_transmit_string("ADC module calibration. Which Gain value would you like to set? \r\n\r\n");
			LPUART1_transmit_string("> ");

			while (state == 1)
			{
				gain = LPUART1_receive_int();								/* Receive Gain Value */

				if ((gain >= 0) && (gain <= 1023))							/* Gain Value validation */
				{
					state = 2;
					LPUART1_transmit_string("ADC module calibration. Which Offset value would you like to set?\r\n\r\n");
					LPUART1_transmit_string("> ");

					while (state == 2)
					{
						offset = LPUART1_receive_int();						/* Receive Offset Value */

						if ((offset >= 0) && (offset <= 255))				/* Offset Value validation */
						{
							state = 0;
							ADC_calibration_init(gain, offset);				/* Convert Channel AD12 to pot on EVB */
							ADC_channel_convert(12);                   		/* Convert Channel AD12 to pot on EVB */
							while(ADC_conversion_complete() == 0){}         /* Wait for conversion complete flag */
							adc_mV_result = ADC_channel_read();       		/* Get channel's conversion results in mV */

							/* Send ADC result by UART */
							LPUART1_transmit_string("ADC result with calibration is: ");
							LPUART1_int_to_char(adc_mV_result);				/* Convert data from int to char to be able to send by UART */
							LPUART1_transmit_string(" mV with UG = ");
							LPUART1_int_to_char(gain);						/* Convert data from int to char to be able to send by UART */
							LPUART1_transmit_string(" and USR_OFS = ");
							LPUART1_int_to_char(offset);					/* Convert data from int to char to be able to send by UART */
							LPUART1_transmit_string("\r\n\r\n");

							WDOG_init(); 									/* Reboot MCU to erase the ADC calibration register */
							Enable_Interrupt(WDOG_EWM_IRQn);				/* Enable WDOG interrupt vector */
						}
						else
						{
							/* Incorrect answer. Invalid Offset Value */
							LPUART1_transmit_string("\r\n");
							LPUART1_transmit_string("Incorrect Offset Value. Try again.\r\n\r\n");
							LPUART1_transmit_string("> ");
						}
					}
				}
				else
				{
					/* Incorrect answer. Invalid Gain Value */
					LPUART1_transmit_string("\r\n");
					LPUART1_transmit_string("Incorrect Gain Value. Try again.\r\n\r\n");
					LPUART1_transmit_string("> ");
				}
			}
		}

		/* ADC module without calibration */
		else if(answer == 'n')
		{
			ADC_init();											/* ADC initialization without calibration */
			ADC_channel_convert(12);                   			/* Convert Channel AD12 to pot on EVB */
			while(ADC_conversion_complete() == 0){}            	/* Wait for conversion complete flag */
			adc_mV_result = ADC_channel_read();       			/* Get channel's conversion results in mV */

			/* Send ADC result by UART */
			LPUART1_transmit_string("\r\n\r\n");
			LPUART1_transmit_string("ADC result without calibration is: ");
			LPUART1_int_to_char(adc_mV_result);					/* Convert data from int to char to be able to send by UART */
			LPUART1_transmit_string(" mV\r\n\r\n");

			WDOG_init();										/* Reboot MCU to erase the ADC calibration register */
			Enable_Interrupt(WDOG_EWM_IRQn);	/* Enable WDOG interrupt vector */
		}

		/* Incorrect answer. Input different of y/n */
		else
		{
			LPUART1_transmit_string("\r\n\r\n");
			LPUART1_transmit_string("Incorrect input. Try again.\r\n\r\n");
			LPUART1_transmit_string("> ");
		}

	}

	return 0;
}

void WDOG_EWM_IRQHandler (void)
{
    /* WDOG interrupt flag active */
	if((WDOG -> CS & WDOG_CS_FLG_MASK) == WDOG_CS_FLG_MASK)
	{
    	WDOG -> CS |= WDOG_CS_FLG_MASK;      							/* Clear the flag */
    }
}
