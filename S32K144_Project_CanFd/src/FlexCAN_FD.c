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

uint32_t  RxCODE;              /* Received message buffer code */
uint32_t  RxID;                /* Received message ID */
uint32_t  RxLENGTH;            /* Recieved message number of data bytes */
uint32_t  RxDATA[2];           /* Received message data (2 words) */
uint32_t  RxTIMESTAMP;         /* Received message time */

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

