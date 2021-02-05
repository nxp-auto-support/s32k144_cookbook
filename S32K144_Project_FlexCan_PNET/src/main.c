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
 * ============================================================================================
 * Usage example of the FlexCAN's Pretended Networking (PNET) mode, for an alternating reception
 * and transmission of CAN Classic frames between two S32K1xx EVBs.
 * Message Buffer number 0 transmits frames with a 8-byte payload, and Wake Up Message Buffer 0 is
 * set up for receiving them at 250 Kbit/s; a green LED is toggled each time 1000 frames
 * are received by each board.
 *
 * Instructions:
 * Build the project, flash one of a pair of EVBs, then uncomment the BOARD_B macro located in line 56
 * and comment out the BOARD_A one, lastly, flash the last board. Perform CAN connections between
 * CAN_LO, CAN_HI and GND, set jumper J107 to 1-2 position and connect a 12V supply,
 * refer to the corresponding Cookbook documentation for detailed images.
 *
 * */

#include "CAN_PNET.h"
#include "register_bit_fields.h"
#include "clocks_and_modes_flexcan.h"
#include "stdint.h"

/* Uncomment BOARD_A or BOARD_B and flash its corresponding profile */
#define BOARD_A
//#define BOARD_B

#define PTD16 (16)										/* Green LED */

/* Counter for the number of frames transmitted */
volatile uint32_t frame_count = 0;

/* Reception frame */
volatile frame_t Reception_frame;

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
	* PTD16			 	| GPIO [GREEN_LED]
	* PTE4              | CAN0 [Rx]
	* PTE5			 	| CAN0 [Tx]
	*/

	/* Pin multiplexing for FlexCAN */
    PCC -> PCC_PORTE_b.CGC = PCC_PCC_PORTE_CGC_1;   	/* Clock gating to PORT E */
    PORTE -> PORTE_PCR4_b.MUX = PORTE_PCR4_MUX_101; 	/* CAN0_RX at PORT E pin 4 */
    PORTE -> PORTE_PCR5_b.MUX = PORTE_PCR5_MUX_101; 	/* CAN0_TX at PORT E pin 5 */

    /* Pin multiplexing for Green LED */
    PCC -> PCC_PORTD_b.CGC = PCC_PCC_PORTD_CGC_1; 		/* Clock gating to PORT D*/
    PORTD -> PORTD_PCR16_b.MUX = PORTD_PCR16_MUX_001; 	/* GPIO multiplexing */
    PTD -> GPIOD_PDDR |= 1 << PTD16; 					/* Direction as output */
}

void Enable_Interrupt (uint8_t vector_number)
{
	S32_NVIC -> NVICISER2 = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
	S32_NVIC -> NVICICPR2 = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

int main (void)
{
	/*!
	 * Initialization:
	 */
	SOSC_init_8MHz ();					/* Initialize system oscilator for 8 MHz xtal */
	SPLL_init_160MHz ();				/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	Normal_RUN_init ();					/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();           				/* Configure ports */
	Stop_Mode_Setup();					/* Start the peripheral */

    /* Instantiate the frame that is going to be transmitted */
	frame_t Transmission_frame;

	/* Definition of the the CAN standard ID and payload of the frame to be transmitted
	*  The IDs were arbitrarily chosen, in CAN Classic, IDs are 11 bits long at most.
	*/
	#if defined(BOARD_A)
		Transmission_frame.ID = 0x1E;
	#elif defined(BOARD_B)
		Transmission_frame.ID = 0xE7;
	#endif

	/* Fill the desired payload to be sent, maximum 8 bytes for CAN Classic */
	Transmission_frame.payload[0] = 0x11223344;
	Transmission_frame.payload[1] = 0x55667788;

	/* Define the frame's ID that the board is going to receive.
	*  Note that the IDs are swapped with respect to the transmission IDs.
	*/
	#if defined(BOARD_A)
		uint32_t ID = 0xE7;
	#elif defined(BOARD_B)
		uint32_t ID = 0x1E;
	#endif

	/* Status variable for validation */
	status_t status;

	/* Start the peripheral and install the specified ID of the destination board */
	status = FlexCAN_init_PNET();

	/* Enable NVIC interrupt */
	Enable_Interrupt(CAN0_Wake_Up_IRQn);

	/* Install the specified ID of the destination board */
	if( status )
		status = FlexCAN_install_ID(ID);

	#if defined(BOARD_A)
		/* Toggle LED initially so it turns on complementary in each board */
		PTD -> GPIOD_PTOR |= 1 << PTD16;

		/* BOARD_A kickstarts the transmission */
		if( status )
			FlexCAN_transmit_frame(&Transmission_frame);
	#endif

	/*!
	* Super-loop for transmitting a frame only when one is received
	*/
    for(;;)
    {
        /* Transition from current Normal Run to STOP2 mode with the WFI (Wait for Interrupts) instruction */
        __asm volatile("WFI");

        /* Transmit a frame back after waking up and executing the reception ISR */
        status = FlexCAN_transmit_frame(&Transmission_frame);

    }
    return 0;
}


/*!
* @brief Interrupt handler that will be executed when FlexCAN wakes up the core after it
* 		 receives a frame that passes the configured filtering. The Wake Up Message
* 		 Buffer (WMB) 0th is used for RX. There are a total of 4 WMBs available,
*        regular MB and WMB are located in independent registers.
*/
void CAN0_Wake_Up_IRQHandler (void)
{
	/* Poll for Low Power mode exit before accessing PN mode registers */
    while(CAN0 -> CAN0_MCR_b.LPMACK);

    /* Read the fields of the received frame */
    Reception_frame.ID  = CAN0 -> CAN0_WMB0_ID >> 18;
    Reception_frame.payload[0] = CAN0 -> CAN0_WMB0_D03;
    Reception_frame.payload[1] = CAN0 -> CAN0_WMB0_D47;

    /* Clear the Wake-Up by match flag bit (W1C) */
    CAN0 -> CAN0_WU_MTC_b.WUMF = 1;

    /* Increment the reception counter */
    frame_count++;

    /* Each 1000 frames received, the green LED will toggle and counter resets */
    if(frame_count == 1000)
    {
    	frame_count = 0;
    	PTD -> GPIOD_PTOR |= 1 << PTD16;
    }

    return;
}
