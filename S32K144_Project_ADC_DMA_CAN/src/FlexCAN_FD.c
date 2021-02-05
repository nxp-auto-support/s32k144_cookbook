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

#include "device_registers.h"	/* include peripheral declarations S32K144 */
#include "FlexCAN_FD.h"
#include <stdio.h>

uint32_t  RxCODE;              /* Received message buffer code */
uint32_t  RxID;                /* Received message ID */
uint32_t  RxLENGTH;            /* Received message number of data bytes */
uint32_t  RxDATA[2];           /* Received message data (2 words) */
uint32_t  RxTIMESTAMP;         /* Received message time */
uint32_t  RxDATA8 [8];         /* Received message data (8 words) */
uint32_t  RxDATA32[8];         /* Received message data (8 words) */

void FLEXCAN0_init(void) {
#define MSG_BUF_SIZE  18    /* Msg Buffer Size. (2 words hdr + 16 words data  = 18 words) */
	uint32_t   i=0;

	PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK; /* CGC=1: enable clock to FlexCAN0 */
	CAN0->MCR |= CAN_MCR_MDIS_MASK;         /* MDIS=1: Disable module before selecting clock */
	CAN0->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;  /* CLKSRC=1: Clock Source = BUSCLK (40 MHz) */
	CAN0->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
	while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}

	/* Good practice: wait for FRZACK=1 on freeze mode entry/exit */

	CAN0->CBT =		CAN_CBT_EPSEG2(15)		/* Configure nominal phase: 500 KHz bit time, 40 MHz Sclock */
							|CAN_CBT_EPSEG1(15)		/* Prescaler = CANCLK / Sclock = 40 MHz / 40 MHz = 1 */
							|CAN_CBT_EPROPSEG(47)	/* EPRESDIV = Prescaler - 1 = 0 */
							|CAN_CBT_ERJW(15);   	/* EPSEG2 = 15 */
													/* EPSEG1 = 15 */
													/* EPROPSEG = 47 */
													/* ERJW = 15 */
													/* BITRATEn =Fcanclk /( [(1 + (EPSEG1+1) + (EPSEG2+1) + (EPROPSEG + 1)] x (EPRESDIV+1)) */
													/*         = 40 MHz /( [(1 + ( 15   +1) + ( 15   +1) + (   46    + 1)] x (    0   +1)) */
													/*          = 40 MHz /( [1+16+16+47] x 1) = 40 MHz /(80x1) = 500 Kz */

    /* Configure data phase: 2 MHz bit time, 40 MHz Sclock */
	CAN0->FDCBT = 	CAN_CBT_EPSEG2(3)/* Prescaler = CANCLK / Sclock = 40 MHz / 40 MHz = 1 */
							|CAN_CBT_EPSEG1(7)/* FPRESDIV = Prescaler - 1 = 0 */
							|CAN_CBT_EPROPSEG(7)/* FPSEG2 = 3 */
							|CAN_CBT_ERJW(3);/* FPSEG1 = 7 */
	/* FPROPSEG = 7 */
	/* FRJW = 3 */
	/* BITRATEf =Fcanclk /( [(1 + (FPSEG1+1) + (FPSEG2+1) + (FPROPSEG)] x (FPRESDIV+!)) */
	/*          = 40 MHz /( [(1 + (  7   +1) + (  3   +1) + (   7    )] x (    0   +1)) */
	/*          = 40 MHz /( [1+8+4+7] x 1) = 40 MHz /(20x1) = 40 MHz / 20 = 2 MHz  */


	CAN0->FDCTRL =	CAN_FDCTRL_FDRATE_MASK	/* Configure bit rate switch, data size, transcv'r delay  */
			|CAN_FDCTRL_MBDSR0(3)	/* BRS=1: enable Bit Rate Swtich in frame's header */
			|CAN_FDCTRL_TDCEN_MASK	/* MBDSR1: Not applicable */
			|CAN_FDCTRL_TDCOFF(5);   /* MBDSR0=3: Region 0 has 64 bytes data in frame's payload */
	/* TDCEN=1: enable Transceiver Delay Compensation */
	/* TDCOFF=5: 5 CAN clocks (300us) offset used */

	for(i=0; i<128; i++ ) {    /* CAN0: clear 128 words RAM in FlexCAN 0 */
		CAN0->RAMn[i] = 0;       /* Clear msg buf words. All buffers CODE=0 (inactive) */
	}
	for(i=0; i<16; i++ ) {          /* In FRZ mode, init CAN0 16 msg buf filters */
		CAN0->RXIMR[i] = 0xFFFFFFFF;  /* Check all ID bits for incoming messages */
	}
	CAN0->RXMGMASK = 0x1FFFFFFF;  /* Global acceptance mask: check all ID bits */

	/* Message Buffer 4 - receive setup: */
	CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] = 0xC4000000; /* Msg Buf 4, word 0: Enable for reception */
	/* EDL=1: Extended Data Length for CAN FD */
	/* BRS = 1: Bit Rate Switch enabled */
	/* ESI = 0: Error state */
	/* CODE=4: MB set to RX inactive */
	/* IDE=0: Standard ID */
	/* SRR, RTR, TIME STAMP = 0: not applicable */
#ifdef NODE_A                                   /* Node A receives msg with std ID 0x511 */
	CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x14440000; /* Msg Buf 4, word 1: Standard ID = 0x511 */
#else                                           /* Node B to receive msg with std ID 0x555 */
	CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x15540000; /* Msg Buf 4, word 1: Standard ID = 0x555 */
#endif
	/* PRIO = 0: CANFD not used */
	CAN0->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK;       /* Enable CRC fix for ISO CAN FD */
	CAN0->MCR = 0x0000081F;       /* Negate FlexCAN 1 halt state & enable CAN FD for 32 MBs */
	while ((CAN0->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
	/* Good practice: wait for FRZACK to clear (not in freeze mode) */
	while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
	/* Good practice: wait for NOTRDY to clear (module ready)  */
}

void FLEXCAN0_transmit_msg(void) { 

/*! 
 *  Start transmition
 *  ===============
 */

/* Assumption:  Message buffer CODE is INACTIVE */
	CAN0->IFLAG1 = 0x00000001;       /* Clear CAN 0 MB 0 flag without clearing others*/
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 2] = 0xA5112233; /* MB0 word 2: data word 0 */
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 3] = 0x44556677; /* MB0 word 3: data word 1 */
#ifdef NODE_A
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 1] = 0x15540000; /* MB0 word 1: Tx msg with STD ID 0x555 */
#else
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 1] = 0x14440000; /* MB0 word 1: Tx msg with STD ID 0x511 */
#endif
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] = 0xCC4F0000 | 8 <<CAN_WMBn_CS_DLC_SHIFT; /* MB0 word 0: */
	/* EDL=1 CAN FD format frame*/
	/* BRS=1: Bit rate is switched inside msg */
	/* ESI=0: ??? */
	/* CODE=0xC: Activate msg buf to transmit */
	/* IDE=0: Standard ID */
	/* SRR=1 Tx frame (not req'd for std ID) */
	/* RTR = 0: data, not remote tx request frame*/
	/* DLC=15; 64 bytes */
}

void FLEXCAN0_receive_msg(void) {  
/*! Receive msg from ID 0x556 using msg buffer 4 
 *  ============================================
 */
	uint8_t j;
	uint32_t dummy;

	RxCODE   = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;  /* Read CODE field */
	RxID     = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK)  >> CAN_WMBn_ID_ID_SHIFT ;
	RxLENGTH = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;
	for (j=0; j<2; j++) {  /* Read two words of data (8 bytes) */
		RxDATA[j] = CAN0->RAMn[ 4*MSG_BUF_SIZE + 2 + j];
	}
	RxTIMESTAMP = (CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] & 0x000FFFF);
	dummy = CAN0->TIMER;             /* Read TIMER to unlock message buffers */
	CAN0->IFLAG1 = 0x00000010;       /* Clear CAN 0 MB 4 flag without clearing others*/
}



/*!
* @brief CAN0 Initialization for padding
*/
void FLEXCAN0_padding_init (void)
{
	#define MSG_BUF_SIZE (10) 								/* Message Buffer Size. (2 words hdr + 8 words data = 10 words) */
	uint32_t i = 0;											/* Counter */

	PCC -> PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK; 	/* CGC = 1 Enable clock to FLEXCAN0 */
	CAN0 -> MCR |= CAN_MCR_MDIS_MASK;         				/* MDIS = 1 Disable module before selecting clock */
	CAN0 -> CTRL1 |= CAN_CTRL1_CLKSRC_MASK;  				/* CLKSRC = 1 Clock Source = BUSCLK (40 MHz) */
	CAN0 -> MCR &= ~CAN_MCR_MDIS_MASK;        				/* MDIS = 0 Enable module configuration (Sets FRZ, HALT) */

	/* Good practice: Wait for FRZACK = 1 on freeze mode entry/exit */
	while (!((CAN0 -> MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)){}

	CAN0 -> CBT = 0x802FB9EF;  			 					/* Configure nominal phase: 500 KHz bit time, 40 MHz Sclock */

	/* BITRATEn = fCANCLK / ([(1 + (EPSEG1 + 1) + (EPSEG2 + 1) + (EPROPSEG + 1)] * (EPRESDIV + 1)) */
	/*          = 40 MHz /  ([(1 + (15 + 1) + (15 + 1) + (46 + 1)] * (0 +1)) */
	/*          = 40 MHz /  ([1 + 16 + 16 + 47] * 1) = 40 MHz / (80 * 1) = 500 KHz */

	CAN0 -> FDCBT = 0x00131CE3; 							/* Configure data phase: 2 MHz bit time, 40 MHz Sclock */

	CAN0 -> FDCTRL = 0x80008500
			       | 2 << CAN_FDCTRL_MBDSR0_SHIFT; 			/* Selects 32 bytes per message buffer */


	for (i = 0; i < 128; i++)								/* Clear message buffer words. All buffers CODE = 0 (Inactive) */
	{
		CAN0 -> RAMn[i] = 0; 								/* CAN0 Clear 128 words RAM in FLEXCAN0 */
	}

	for (i=0; i<16; i++ )  									/* In FRZ mode, initialize CAN0 16 message buffer filters */
	{
		CAN0 -> RXIMR[i] = 0xFFFFFFFF;  					/* Check all ID bits for incoming messages */
	}

	CAN0 -> RXMGMASK = 0x1FFFFFFF;  						/* Global acceptance mask. Check all ID bits */

	#ifndef NODE_A											/* Node A receives message with Standard ID = 0x511 */

		/* Message Buffer 0 - receive setup: */
		CAN0 -> RAMn[0 * MSG_BUF_SIZE + 0] = 0xC4000000; 	/* Message Buffer 0, word 0: Enable for reception */
		CAN0 -> RAMn[0 * MSG_BUF_SIZE + 1] = 0x43C << 18; 	/* Message Buffer 0, word 1: Standard ID = 0x511 */

		/* Message Buffer 4 - receive setup: */
		CAN0 -> RAMn[4 * MSG_BUF_SIZE + 0] = 0xC4000000; 	/* Message Buffer 4, word 0: Enable for reception */
		CAN0 -> RAMn[4 * MSG_BUF_SIZE + 1] = 0x36F << 18; 	/* Message Buffer 4, word 1: Standard ID = 0x511 */

	#endif

	CAN0 -> CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK;       		/* Enable CRC fix for ISO CAN FD */
	CAN0 -> MCR = 0x0000081F;      		 					/* Negate FlexCAN 1 halt state & enable CAN FD for 32 MBs */

	/* Good practice: Wait for FRZACK to clear (not in freeze mode) */
	while ((CAN0 -> MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT){}

	/* Good practice: Wait for NOTRDY to clear (module ready) */
	while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
}

/*!
* @brief FlexCAN Tx Process (MB0)
*/
void FLEXCAN0_transmit_8 (void)
{
	/* Assumption: Message Buffer CODE is INACTIVE */
	CAN0 -> IFLAG1 = 0x00000001;       						/* Clear CAN0 MB 0 flag without clearing others */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 2] = 0x11111111; 		/* MB0 word 2: Data word 0 */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 3] = 0x11111111; 		/* MB0 word 3: Data word 1 */

	/* Software Padding */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 4] = 0xCCCCCCCC; 		/* MB0 word 4: Data word 2 */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 5] = 0xCCCCCCCC; 		/* MB0 word 5: Data word 3 */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 6] = 0xCCCCCCCC; 		/* MB0 word 6: Data word 4 */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 7] = 0xCCCCCCCC; 		/* MB0 word 7: Data word 5 */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 8] = 0xCCCCCCCC; 		/* MB0 word 8: Data word 6 */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 9] = 0xCCCCCCCC; 		/* MB0 word 9: Data word 7 */

	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 1] = 0x43C << 18; 		/* MB0 word 1: Tx message with Standard ID = 0x555 */
	CAN0 -> RAMn[0 * MSG_BUF_SIZE + 0] = 0xCC400000 | 13 << CAN_WMBn_CS_DLC_SHIFT; /* MB0 word 0: */
}

/*!
* @brief FlexCAN Rx process (MB0)
*/
void FLEXCAN0_receive_8 (void)
{
	char buffer[200];
	uint8_t j;

	for (j = 0; j < 8; j++) 								/* Read 8 words of data (8 bytes) */
	{
		RxDATA8[j] = CAN0 -> RAMn[0 * MSG_BUF_SIZE + 2 + j];
	}

	CAN0 -> IFLAG1 = 0x00000001; 							/* Clear CAN0 MB 0 flag without clearing others */

//	snprintf(buffer, 200, "MB0: %10X %10X %10X %10X %10X %10X %10X %10X\n\r",
//    RxDATA8[0], RxDATA8[1], RxDATA8[2], RxDATA8[3], RxDATA8[4], RxDATA8[5], RxDATA8[6], RxDATA8[7]);
//	print(buffer);
}

/*!
* @brief FlexCAN Tx process (MB4)
*/
void FLEXCAN0_transmit_32 (void)
{
	/* Assumption: Message Buffer CODE is INACTIVE */
	CAN0 -> IFLAG1 = 0x00000010;       						/* Clear CAN0 MB 4 flag without clearing others */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 2] = 0x22222222; 		/* MB4 word 2: Data word 0 */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 3] = 0x22222222; 		/* MB4 word 3: Data word 1 */

	/* Software Padding */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 4] = 0x22222222; 		/* MB4 word 4: Data word 2 */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 5] = 0x22222222; 		/* MB4 word 5: Data word 3 */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 6] = 0x22222222; 		/* MB4 word 6: Data word 4 */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 7] = 0x22222222; 		/* MB4 word 7: Data word 5 */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 8] = 0x22222222; 		/* MB4 word 8: Data word 6 */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 9] = 0x22222222; 		/* MB4 word 9: Data word 7 */

	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 1] = 0x36F << 18;  		/* MB0 word 1: Tx message with Standard ID = 0x555 */
	CAN0 -> RAMn[4 * MSG_BUF_SIZE + 0] = 0xCC400000 | 13 << CAN_WMBn_CS_DLC_SHIFT; /* MB0 word 0: */
}

/*!
* @brief FlexCAN Rx process (MB0)
*/
void FLEXCAN0_receive_32 (void)
{
	char buffer[200];
	uint8_t j;

	for (j = 0; j < 8; j++)   								/* Read 8 words of data (8 bytes) */
	{
		RxDATA32[j] = CAN0 -> RAMn[4 * MSG_BUF_SIZE + 2 + j];
	}

	CAN0 -> IFLAG1 = 0x00000010; 							/* Clear CAN0 MB 4 flag without clearing others */

//	snprintf(buffer, 200, "MB4: %10X %10X %10X %10X %10X %10X %10X %10X\n\r",
//    RxDATA32[0], RxDATA32[1], RxDATA32[2], RxDATA32[3], RxDATA32[4], RxDATA32[5], RxDATA32[6], RxDATA32[7]);
//	print(buffer);
}


/*! Function to setup the configuration of the Flexcan_FD in each of the
 * boards that are being used to test this project, where the board assigned to NODE_1
 * has an ID of 0x511 and will ask the board 2 (which is assigned to Node_2 and has an ID
 * of 0x555) about certain information (may be ADC value of pot or pin)
 */
void FLEXCAN_FD_Config(void){
	PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for CAN */
	CAN0->MCR |= CAN_MCR_MDIS_MASK;						/* Disable module for clock selection */
	CAN0->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;				/* Clock source is Bus Clock */
	CAN0->MCR &=~ CAN_MCR_MDIS_MASK;					/* Enable module */

	CAN0->MCR &=~ CAN_MCR_FRZACK_MASK;										/* Enable Freeze Mode for clk Configuration */
	while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT));	/* Wait for Freeze Mode */


	CAN0->CBT = CAN_CBT_BTF_MASK|		/* Enable extended CAN */
				CAN_CBT_EPRESDIV(1)|	/* Sclock frequency = Fcanclk / (EPRESDIV + 1) */
				CAN_CBT_ERJW(15)|		/* Resync Jump Width = ERJW + 1 */
				CAN_CBT_EPROPSEG(46)|	/* Propagation Segment Time = (EPROPSEG + 1) × Time-Quanta */
				CAN_CBT_EPSEG1(15)|		/* Phase Buffer Segment 1 = (EPSEG1 + 1) × Time-Quanta */
				CAN_CBT_EPSEG2(15);		/* Phase Buffer Segment 1 = (EPSEG2 + 1) × Time-Quanta */
	/* BITRATEn =Fcanclk /( [(1 + (EPSEG1+1) + (EPSEG2+1) + (EPROPSEG + 1)] x (EPRESDIV+1)) */

	CAN0->FDCBT = CAN_FDCBT_FPRESDIV(1)|	/* Sclock frequency = PE clock frequency / (FPRESDIV + 1) */
				  CAN_FDCBT_FRJW(3)|		/* Resync Jump Width = FSJW + 1 */
				  CAN_FDCBT_FPROPSEG(7)|	/* Time-Quantum = one Sclock period */
				  CAN_FDCBT_FPSEG1(7)|		/* Phase Segment 1 = (FPSEG1 + 1) × Time-Quanta */
				  CAN_FDCBT_FPSEG2(3);		/* Phase Segment 2 = (FPSEG2 + 1) × Time-Quanta */
	/* BITRATEf =Fcanclk /( [(1 + (FPSEG1+1) + (FPSEG2+1) + (FPROPSEG)] x (FPRESDIV+1)) */

	CAN0->FDCTRL = CAN_FDCTRL_FDRATE_MASK|	/* Rate Switch Enable */
				   CAN_FDCTRL_MBDSR0(0b11)|	/* Message buffer size of 64 bytes */
				   CAN_FDCTRL_TDCEN_MASK|	/* Transceiver Delay Compensation is Enable */
				   CAN_FDCTRL_TDCOFF(5);	/* 300us offset use */

	for(count = 0; count < 128; count++){
		CAN0->RAMn[count] = 0;				/* Clear all the buffer */
	}

	CAN0->RXMGMASK = 0x1FFFFFFF;			/* Global ID */
	for(count = 0; count < 16; count++){
		CAN0->RXIMR[count] = 0xFFFFFFFF;	/* Buffers IDs */
	}

	CAN0->RAMn[ 4*MsgBuffSize+0] = 1<<31|		/* Extended data length for CAN FD */
								   1<<30|		/* Bit rate switching for CAN FD */
								   0b0100<<24;	/* Message buffer is active and empty */

	#ifdef Node_1
	CAN0->RAMn[ 4*MsgBuffSize+1] = 0x511<<18;	/* Node 1 receives messages with 0x511ID */
	#endif
	#ifdef Node_2
	CAN0->RAMn[ 4*MsgBuffSize+1] = 0x555<<18;	/* Node 2 receives messages with 0x555ID */
	#endif

	CAN0->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK;	/* ISO CAN FD Enable */
	CAN0->MCR = CAN_MCR_FDEN_MASK|				/* CAN FD is Enable */
				CAN_MCR_MAXMB(0b11111);			/* Number Of The Last Message Buffer */

	CAN0->IMASK1 = 0x10;							/* Enable Interruption */
	S32_NVIC->ICPR[2] = 1<<(CAN0_ORed_0_15_MB_IRQn  % 32);	/* Clear any pending IR for CAN*/
	S32_NVIC->ISER[2] = 1<<(CAN0_ORed_0_15_MB_IRQn  % 32);	/* Enable IRQ for CAN*/
	S32_NVIC->IP[CAN0_ORed_0_15_MB_IRQn ] = 5;				/* Level of priority (0 - 15) */

	while((CAN0->MCR && CAN_MCR_FRZACK_MASK)>>CAN_MCR_FRZACK_SHIFT);	/* Wait to exit from Freeze mode */
	while((CAN0->MCR && CAN_MCR_NOTRDY_MASK)>>CAN_MCR_NOTRDY_SHIFT);	/* Wait for CAN mode to be ready */
}
