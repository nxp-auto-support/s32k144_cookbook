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
 * =============================================================================================
 * You will need 2 boards for this project to work properly as you have to set up one of them as Node 1
 * and the other as Node 2 to cover all the functionality of the project. Node 1 is configured so it sends
 * a CAN petition to Node 2 asking for the ADC value of the potentiometer or a PIN depending on which switch
 * is pressed down in Node 1. Meanwhile Node 2 is configured to constantly measure each ADC (Potentiometer
 * and Pin) and save the measurements in an array through DMA requests after the COCO flag of the ADC is
 * activated, when Node 2 receives a request from Node 1 it starts a transmission through CAN to send
 * the last measures to Node 1.
 *
 * The PIN is PTB0.
 *
 * */

#include "device_registers.h"
#include "clocks_and_modes.h"
#include "dma.h"
#include "ADC.h"
#include "pdb.h"
#include "FlexCAN_FD.h"

#define Condition1_Pin12 ((PORTC->PCR[12] & PORT_PCR_ISF_MASK)>>PORT_PCR_ISF_SHIFT)		/* Used to check if the interruption flag of PTC12 is activated */
#define Condition2_Pin12 ((PORTC->PCR[12] & PORT_PCR_IRQC_MASK)>>PORT_PCR_IRQC_SHIFT)	/* Used to check if the interruption config. has changed for PTC12 */
#define Condition1_Pin13 ((PORTC->PCR[13] & PORT_PCR_ISF_MASK)>>PORT_PCR_ISF_SHIFT)		/* Used to check if the interruption flag of PTC13 is activated */
#define Condition2_Pin13 ((PORTC->PCR[13] & PORT_PCR_IRQC_MASK)>>PORT_PCR_IRQC_SHIFT)	/* Used to Check if the interruption config. has changed for PTC13*/

uint32_t ValuePOT;			/* Variable to save the Value of the POT received by Node_1 from Node_2 */
uint32_t ValuePin;			/* Variable to save the Value of the PIN received by Node_1 from Node_2	*/
uint32_t ADC_nodo2[4];		/* TCD Destination Array of the DMA to deposit the ADC0 measurements in Node_2 */

void WDOG_disable (void)
{
	WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
	WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
	WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

void GPIO_Config(void){
	PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable the clock for port E */
	PORTE->PCR[4] |= PORT_PCR_MUX(5);					/* Selected PE4 as Rx CAN */
	PORTE->PCR[5] |= PORT_PCR_MUX(5);					/* Selected PE5 as Tx CAN */

	#ifdef Node_1
		PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable the clock for port C */
		PORTC->PCR[12] |= PORT_PCR_MUX(1)|					/* Selected PC12 as GPIO */
						  PORT_PCR_IRQC(0b1001);			/* Enable interruption in rising edge */
		PTC->PDDR &=~ 1<<12;								/* Pin is configured as input */

		PORTC->PCR[13] |= PORT_PCR_MUX(1)|					/* Selected PC13 as GPIO */
						  PORT_PCR_IRQC(0b1001);			/* Enable interruption in rising edge */
		PTC->PDDR &=~ 1<<13;								/* Pin is configured as input */

		S32_NVIC->ICPR[PORTC_IRQn>>5] = 1<<(PORTC_IRQn &0x1F);		/* Clear any pending IR for PortC*/
		S32_NVIC->ISER[PORTC_IRQn>>5] = 1<<(PORTC_IRQn &0x1F);		/* Enable IRQ for Port C*/
		S32_NVIC->IP[PORTC_IRQn] = 1;								/* Level of priority (0 - 15) */

	#endif
}

int main(void) {
	WDOG_disable();        /* Disable WDOG */
	SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
	SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
    GPIO_Config();		  	/* Configure PINs to work for CANFD and set up the interruption if Node_1 is defined */
    FLEXCAN_FD_Config();	/* Initialize FLEXCAN FD if Node_1 is defined ID = 0x5111 else if Node_2 is define ID = 0x555 */

	#ifdef Node_2
		DMA_Config(ADC_nodo2);	/* Set up DMAMUX CH3 with ADC0 COCO requests and set up the TCD Source and Destination */
		ADC_calibration_init(0,0);		/* Starts ADC calibration	*/
		ADC_Config(12);			/* Set up the ADC0 Channels to be used by HW Trigger of the PDB */
		PDB_Config();			/* Set up the PDB to trigger ADC */
	#endif


	for (;;) {
    }
    return 0;
}

/* !
 * Interruptions:
 * ===========================================================================================
 */

#ifdef Node_1
/******************************************************************************
 * When one of the switch is pressed down an interrupt is enabled and a CAN
 * transmission is executed to request the value of the ADC on
 * node 2, depending on the switch the request could be the Pot
 * or a Pin (POT = SWITCH_2 and PIN=SWITCH_3)
 *****************************************************************************/
void PORTC_IRQHandler(void){
	if(Condition1_Pin12 && Condition2_Pin12){	/* POT Measure */
		PORTC->PCR[12] |= PORT_PCR_ISF_MASK;	/* Turn off flag of interruption */
		CAN0->IFLAG1 = CAN_IFLAG1_BUF0I_MASK;	/* Clear MB0 Flag */

		CAN0->RAMn[ 0*MsgBuffSize +2] = 0xA;			/* Message word 1 */
		CAN0->RAMn[ 0*MsgBuffSize +3] = 0x87654321;	/* Message word 2 */
		CAN0->RAMn[ 0*MsgBuffSize +1] = 0x555<<18;	/* ID receiver */
		CAN0->RAMn[ 0*MsgBuffSize +0] = 1<<31|		/* CAN FD format */
										   1<<30|		/* Bit Rate switch */
										   0b1100<<24|	/* Activate buffer to transmit */
										   1<<22|		/* ??? */
										   0b1111<<16|	/* 64 bytes */
										   8 <<CAN_WMBn_CS_DLC_SHIFT;
	}

	if(Condition1_Pin13 && Condition2_Pin13){	/* Pin Measure */
		PORTC->PCR[13] |= PORT_PCR_ISF_MASK;	/* Turn off flag of interruption */
		CAN0->IFLAG1 = CAN_IFLAG1_BUF0I_MASK;	/* Clear MB0 Flag */

		CAN0->RAMn[ 0*MsgBuffSize +2] = 0xB;			/* Message word 1 */
		CAN0->RAMn[ 0*MsgBuffSize +3] = 0x12345678;	/* Message word 2 */
		CAN0->RAMn[ 0*MsgBuffSize +1] = 0x555<<18;	/* ID receiver */
		CAN0->RAMn[ 0*MsgBuffSize +0] = 1<<31|		/* CAN FD format */
										   1<<30|		/* Bit Rate switch */
										   0b1100<<24|	/* Activate buffer to transmit */
										   1<<22|		/* ??? */
										   0b1111<<16|	/* 64 bytes */
										   8 <<CAN_WMBn_CS_DLC_SHIFT;
		}
}
#endif

/******************************************************************************
 * When the receptions ends the interruption is enable,
 * in this interruption the information from the bus is storaged, if
 * the information is coming from Node_1 to Node_2 then a transmission
 * begins from Node_2 to Node_1 with the corresponding value from the
 * ADC.
 *****************************************************************************/
void CAN0_ORed_0_15_MB_IRQHandler(void){
	RexCode = (CAN0->RAMn[ 4*MsgBuffSize + 0] & 0x0F000000) >> 24;								/* Code field */
	RexID = (CAN0->RAMn[ 4*MsgBuffSize + 1] & CAN_WMBn_ID_ID_MASK) >> CAN_WMBn_ID_ID_SHIFT;		/* Message ID */
	RexLength = (CAN0->RAMn[ 4*MsgBuffSize + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;	/* Message length (32 bits) */

	for(count = 0; count < 2; count++){
		RexData[count] = CAN0->RAMn[ 4*MsgBuffSize + 2 + count];	/* Extracting words from the message */
	}

	RexTime = CAN0->TIMER;             /* Read TIMER to unlock message buffers */
	CAN0->IFLAG1 = 0x00000010;      /* Clear CAN0 MB4 flag */



	#ifdef Node_1
		if(RexData[0] == 0xA){	/* Pot Measure */
			ValuePOT = RexData[1];
		}
		if(RexData[0] == 0xB){	/* Pin Measure */
			ValuePin = RexData[1];
		}
	#endif

#ifdef Node_2

	CAN0->IFLAG1 = CAN_IFLAG1_BUF0I_MASK;	/* Clear MB0 Flag */

	if(RexData[0] == 0xA){	/* Pot Measure request */
		CAN0->RAMn[ 0*MsgBuffSize +2] = 0xA;				/* Message word 1 */
		CAN0->RAMn[ 0*MsgBuffSize +3] = ADC_nodo2[1];	/* Message word 2 */
		CAN0->RAMn[ 0*MsgBuffSize +1] = 0x511<<18;		/* ID receiver */
		CAN0->RAMn[ 0*MsgBuffSize +0] = 1<<31|			/* CAB FD format */
										   1<<30|			/* Bit Rate switch */
										   0b1100<<24|		/* Activate buffer to transmit */
										   1<<22|			/* ??? */
										   0b1111<<16|		/* 64 bytes */
										   8 <<CAN_WMBn_CS_DLC_SHIFT;
	}
	if(RexData[0] == 0xB){	/* Pin Measure request */
		CAN0->RAMn[ 0*MsgBuffSize +2] = 0xB;				/* Message word 1 */
		CAN0->RAMn[ 0*MsgBuffSize +3] = ADC_nodo2[2];	/* Message word 2 */
		CAN0->RAMn[ 0*MsgBuffSize +1] = 0x511<<18;		/* ID receiver */
		CAN0->RAMn[ 0*MsgBuffSize +0] = 1<<31|			/* CAB FD format */
										   1<<30|			/* Bit Rate switch */
										   0b1100<<24|		/* Activate buffer to transmit */
										   1<<22|			/* ??? */
										   0b1111<<16|		/* 64 bytes */
										   8 <<CAN_WMBn_CS_DLC_SHIFT;

	}
#endif

}
