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

#ifndef FLEXCAN_FD_H_
#define FLEXCAN_FD_H_

/* UNCOMMENT THE NEXT LINE ON THE  1ST BOARD AND COMMENT IT ON THE 2ND BOARD */
#define Node_1			/* Node 1 request values from the ADC on the Node 2 */
/* UNCOMMENT THE NEXT LINE ON THE 2ND BOARD	AND COMMENT IT ON THE 1ST BOARD	*/
//#define Node_2		/* Node 2 operates the ADC and wait for the request of Node 1 to transmit the values */

#define NODE_A        /* If using 2 boards as 2 nodes, NODE A transmits first to NODE_B */
#define SBC_MC33903   /* SBC requires SPI init + max 1MHz bit rate */

uint32_t RexCode;	/* Message buffer code */
uint32_t RexID;		/* Message ID */
uint32_t RexLength;	/* Number of data bytes */
uint32_t RexData[2];	/* Array where data is storage */
uint32_t RexTime;	/* Message time */
uint8_t count; 		/* Counter for loops */
#define MsgBuffSize 18 /* Msg Buffer Size. (2 words hdr + 16 words data  = 18 words) */

void FLEXCAN0_init 			(void);
void FLEXCAN0_transmit_msg 	(void);
void FLEXCAN0_receive_msg 	(void);
void FLEXCAN0_padding_init 	(void);
void FLEXCAN0_transmit_8 	(void);
void FLEXCAN0_receive_8 	(void);
void FLEXCAN0_transmit_32 	(void);
void FLEXCAN0_receive_32 	(void);
void FLEXCAN_FD_Config(void);


#endif /* FLEXCAN_FD_H_ */
