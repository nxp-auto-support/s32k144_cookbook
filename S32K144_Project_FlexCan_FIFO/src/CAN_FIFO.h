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

#ifndef CAN_FIFO_H_
#define CAN_FIFO_H_

#include "stdint.h"

/* Macro for the maximum transfer unit for CAN Classic frame payload (8 bytes = 2 words) */
#define MAX_MTU_WORDS   (2u)

/*!
* @brief Status codes for the return value status
*/
typedef enum
{
	Failure = -1,
	BufferFull = 0,
	Success = 1
} status_t;


/*!
* @brief Structure for a CAN frame. Header + Payload.
*/
typedef struct
{
	uint32_t ID;
	uint32_t payload[MAX_MTU_WORDS];
} frame_t;


/*!
* @brief FlexCAN functions
*/
status_t FlexCAN_init_RXFIFO		(void);
status_t FlexCAN_install_ID			(uint32_t id);
status_t FlexCAN_transmit_frame		(frame_t* frame);
status_t FlexCAN_receive_frame		(frame_t* frame);

#endif /* CAN_FIFO_H_ */
