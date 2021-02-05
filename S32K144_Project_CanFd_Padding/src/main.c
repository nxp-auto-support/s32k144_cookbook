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
 * ===================================================================================================
 * CAN-FD protocol allows frames with more than 8 data bytes. Received data bytes that exceed the
 * CAN FD's data handling capacity shall be discarded. A such implementation that is requested to
 * transmit a longer frame shall padding up the rest of the DATA-FIELD with a constant byte pattern (e.g $CC).
 *
 * For S32K: "When the DLC value stored in the MB selected for transmission is larger than the respective MB
 * pay-load size, FlexCAN adds the necessary number of bytes with constant 0xCC pattern to complete
 * the expected DLC".
 *
 * Above statement applies when the pre-configured MBDSR0 is lower than the actual transmitted DLC.
 * When the pre-configured MBDSR0 is higher than the actual transmitted DLC. Tx data must be padded
 * manually by software
 *
 * The intention of this project is to show how this manual passing is done, taking the CAN-FD cookbook
 * example as starting point, this code will do the following tasks
 * NODE A (Tx)
 * - Transmits 8-bytes ($11, $11, $11...) with DLC=13 to the ID=$43C, whenever BTN0 is pressed. (Using the MB0).
 * - Transmits 32-bytes ($22, $22, $22...) with DLC=13 to the ID=$36F, whenever BTN1 is pressed. (Using the MB4).
 * NODE B (Rx) (NODE_A macro uncommented-out)
 * - Polls MB0 flag (ID=$43C), when a complete message is received it prints the content of the MB0 (32 bytes).
 * - Polls MB4 flag (ID=$36F), when a complete message is received it prints the content of the MB4 (32 bytes).
 *
 * PTC12 and PTC13 are used to start the transmission frame.
 * The UART is only used to display the message content of the Rx node in the terminal at 9600 baud: TeraTerm or other software.
 * */

#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
#include "LPUART.h"
#include "FlexCAN_FD.h"
#include <stdio.h>

#define PTC6 (6)
#define PTC7 (7)
#define PTC12 (12)
#define PTC13 (13)
#define PTE4 (4)
#define PTE5 (5)

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
	* PTC12             | GPIO [BUTTON_SW2]
	* PTC13				| GPIO [BUTTON_SW3]
	* PTE4              | CAN0 [Rx]
	* PTE5			 	| CAN0 [Tx]
	*/

	PCC -> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; 					/* Enable clock for PORT C */
	PCC -> PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK; 					/* Enable clock for PORT E */

	PORTC -> PCR[PTC12] = PORT_PCR_MUX(1)  								/* Port C12: MUX = GPIO */
						| PORT_PCR_PFE_MASK; 							/* Port C12: MUX = GPIO, input filter enabled */

	PTC -> PDDR &= ~(1<<PTC12);											/* Port C12: Data Direction = input */

	PORTC -> PCR[PTC13] = PORT_PCR_MUX(1)  								/* Port C13: MUX = GPIO */
						| PORT_PCR_PFE_MASK; 							/* Port C13: MUX = GPIO, input filter enabled */

	PTC -> PDDR &= ~(1<<PTC13);											/* Port C13: Data Direction = input */

	PORTC -> PCR[PTC6] |= PORT_PCR_MUX(2);								/* Port C6: MUX = UART1 RX */
	PORTC -> PCR[PTC7] |= PORT_PCR_MUX(2);   							/* Port C7: MUX = UART1 TX */

	PORTE -> PCR[PTE4] |= PORT_PCR_MUX(5);								/* Port E4: MUX = CAN0 RX */
	PORTE -> PCR[PTE5] |= PORT_PCR_MUX(5); 								/* Port E5: MUX = CAN0 TX */
}

void WDOG_disable (void)
{
	WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
	WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
	WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

int main(void)
{
	/*!
	 * Initialization:
	 */
	WDOG_disable();						/* Disable watchdog */
	SOSC_init_8MHz();      				/* Initialize system oscilator for 8 MHz xtal */
	SPLL_init_160MHz();    				/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); 				/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();           				/* Configure ports */
	LPUART1_init();
	FLEXCAN0_padding_init();

	/* Send/Receive CAN frames */
    for(;;)
    {
		#ifdef NODE_A
			if (PTC -> PDIR & (1<<PTC12))
			{
				while(PTC -> PDIR & (1<<PTC12)){}
				FLEXCAN0_transmit_8();
			}
			else if (PTC -> PDIR & (1<<PTC13))
			{
				while(PTC -> PDIR & (1<<PTC13)){}
				FLEXCAN0_transmit_32();
			}

		#else
			if (CAN0 -> IFLAG1 & 1)
			{
				FLEXCAN0_receive_8();
			}
			else if ((CAN0 -> IFLAG1 >> 4) & 1)
			{
				FLEXCAN0_receive_32();
			}

		#endif
    }

	return 0;
}

