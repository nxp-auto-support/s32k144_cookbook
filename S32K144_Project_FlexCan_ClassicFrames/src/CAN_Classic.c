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

#include "CAN_Classic.h"
#include "register_bit_fields.h"
#include "stdint.h"

#define __IOM volatile 							/* The compiler won't optimize this macro */

/*!
* @brief Bit field declaration for the transmission and reception message buffers. See "Message Buffer Structure" in RM.
* 		 NOTE: Since this example manages CAN Classic (8 bytes for header and 8 bytes for payload)
* 		 there are 32 Message Buffers available. The FIFO is not enabled.
*/
typedef struct
{
    struct
	{
      __IOM uint32_t TIMESTAMP  : 16;			/* Note that if you add from "TIMESTAMP" to "PRIO" */
      __IOM uint32_t DLC        : 4;			/* you get 64 bits = 8 bytes for header */
      __IOM uint32_t RTR        : 1;			/* 16 + 4 + 1 + ... + 3 = 64 */
      __IOM uint32_t IDE        : 1;
      __IOM uint32_t SRR        : 1;
            uint32_t            : 1;
      __IOM uint32_t CODE       : 4;
            uint32_t            : 1;
      __IOM uint32_t ESI        : 1;
      __IOM uint32_t BRS        : 1;
      __IOM uint32_t EDL        : 1;
            uint32_t            : 18;
      __IOM uint32_t STD_ID     : 11;
      __IOM uint32_t PRIO       : 3;
      __IOM uint32_t payload[2];				/* 8 bytes (2 words) for payload */
    } Classic_MessageBuffer[32];				/* 32 MBs available */
} CAN0_MB_t;

/*!
* @brief Type casting of the structure declared above to the corresponding CAN0 memory area.
*        Refer to the header file "register_bit_fields.h" located inside the project's
*        "include" folder for further detail.
* 		 0x80 is the corresponding offset where the message buffer's memory area is located.
*/
#define CAN0_MB ((CAN0_MB_t*)(CAN0_BASE + 0x80))


/*!
* @brief Structure for the CAN bit timings. See "Protocol Timing" in RM (FlexCAN Chapter)
*/
typedef struct
{
	uint8_t PRESDIV;
	uint8_t PROPSEG;
	uint8_t PSEG1;
	uint8_t PSEG2;
	uint8_t RJW;
} CAN_bit_timings_t;

/* CAN bit timings for 500 Kbit/s with 16 time quantas */
CAN_bit_timings_t timings =
{
	.PRESDIV = 0,
    .PROPSEG = 5,
    .PSEG1 = 5,
    .PSEG2 = 2,
    .RJW = 2,

	/* (PRESDIV + 1) * (PROPSEG + PSEG1 + PSEG2 + 4) = 16 quantas */
	/* Resynchronization Jump Width = RJW + 1 = 3 */
	/* Bit Timing = FlaxCan CLK / time quantas = 8 MHz / 16 = 500 Kbit/s */
};


/*!
* @brief Enum for the index of the 2 Message Buffers (MB) used, 0th for TX and the 1st for RX.
* 		 There are a total of 32 MBs available, however in this project only 2 MBs are used.
*/
typedef enum
{
    TX_MB = 0,
    RX_MB = 1
} MB_index_Enum;


/*!
* @brief FlexCAN Initialization for Classic Frames transmission and reception at 500 Kbits/s
*
* @return Success If the peripheral was started without errors
*/
status_t FlexCAN_init_Classic (void)
{
	PCC -> PCC_FlexCAN0_b.CGC   = PCC_PCC_FlexCAN0_CGC_1; 	/* FlexCAN0 clock gating */

    /* Set asynchronous clock source SOSCDIV2 for feeding @ 8 MHz to FlexCAN */
    CAN0 -> CAN0_MCR_b.MDIS     = CAN0_MCR_MDIS_1;        	/* Disable FlexCAN module for clock source selection */
    CAN0 -> CAN0_CTRL1_b.CLKSRC = CAN0_CTRL1_CLKSRC_0;  	/* Select SOSCDIV2 as source (8 MHz) */
    CAN0 -> CAN0_MCR_b.MDIS     = CAN0_MCR_MDIS_0;        	/* Enable FlexCAN peripheral */
    CAN0 -> CAN0_MCR_b.HALT     = CAN0_MCR_HALT_1;        	/* Request freeze mode entry */
    CAN0 -> CAN0_MCR_b.FRZ      = CAN0_MCR_FRZ_1;			/* Enter in freeze mode */

    /* Block for freeze mode entry */
    while(!(CAN0 -> CAN0_MCR_b.FRZACK));

    CAN0 -> CAN0_MCR_b.MAXMB  = 1;                 			/* Maximum number of MB's as 2 (one for TX and one for RX) */
    CAN0 -> CAN0_MCR_b.SRXDIS = CAN0_MCR_SRXDIS_1; 			/* Disable self-reception of frames if ID matches */
    CAN0 -> CAN0_MCR_b.IRMQ   = CAN0_MCR_IRMQ_1;   			/* Enable individual message buffer ID masking */

    /* CAN Bit Timing (CBT) configuration for a bit rate of 500 Kbit/s with 16 time quantas, in accordance with Bosch 2012 specification */
    CAN0 -> CAN0_CTRL1_b.PRESDIV = timings.PRESDIV;
    CAN0 -> CAN0_CTRL1_b.PROPSEG = timings.PROPSEG;
    CAN0 -> CAN0_CTRL1_b.PSEG1   = timings.PSEG1;
    CAN0 -> CAN0_CTRL1_b.PSEG2   = timings.PSEG2;
    CAN0 -> CAN0_CTRL1_b.RJW     = timings.RJW;

    /* Exit from freeze mode */
    CAN0 -> CAN0_MCR_b.HALT = CAN0_MCR_HALT_0;
    CAN0 -> CAN0_MCR_b.FRZ  = CAN0_MCR_FRZ_0;

    /* Block for freeze mode exit */
    while(CAN0 -> CAN0_MCR_b.FRZACK);

    /* Block for module ready flag */
    while(CAN0 -> CAN0_MCR_b.NOTRDY);

    /* Success initialization */
    return Success;
}


/*!
* @brief Setup a message buffer for reception of a specific ID
*
* @param [uint32_t id] Standard ID
*
* @return Success If the ID was installed correctly
*/
status_t FlexCAN_install_ID (uint32_t id)
{
    /* Request freeze mode entry */
    CAN0 -> CAN0_MCR_b.HALT = CAN0_MCR_HALT_1;
    CAN0 -> CAN0_MCR_b.FRZ  = CAN0_MCR_FRZ_1;

    /* Block for freeze mode entry */
    while (!(CAN0 -> CAN0_MCR_b.FRZACK));

    /* An "All-care" bits mask 0x7FF is used for the 11 bits of CAN Classic IDs,
     * Extended and Standard IDs are installed in the same register, but the Standard
     * ID section correspond to the 11 most significant bits of the whole 29-bit ID section,
     * thus, requiring a 18 left shift. Refer to "Message Buffer Structure" in RM for further detail */
    CAN0 -> CAN0_RXIMR0_b.MI = 0x7FF << 18;

    /* Configure reception message buffer. See "Message Buffer Structure" in RM */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].EDL =  0;		/* No extended data length */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].BRS =  0;		/* No bit-rate switch */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].ESI =  0;		/* No applies */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].CODE = 4;		/* When a frame is received successfully, this field is automatically updated to FULL */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].SRR =  0;		/* No applies */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].IDE =  0;		/* Standard ID */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].RTR =  0;		/* No remote request made */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].DLC = 0x8;  	/* 8 bytes of payload */

    /* Configure the ID */
    CAN0_MB -> Classic_MessageBuffer[RX_MB].STD_ID = id;

    /* Exit from freeze mode */
    CAN0 -> CAN0_MCR_b.HALT = CAN0_MCR_HALT_0;
    CAN0 -> CAN0_MCR_b.FRZ  = CAN0_MCR_FRZ_0;

    /* Block for freeze mode exit */
    while(CAN0 -> CAN0_MCR_b.FRZACK);

    /* Block for module ready flag */
    while(CAN0 -> CAN0_MCR_b.NOTRDY);

    /* Success ID installation */
    return Success;
}


/*!
* @brief Transmit a single CAN frame
*
* @param [frame] 	 The reference to the frame that is going to be transmitted
*
* @return Success    If the frame was sent immediately
*/
status_t FlexCAN_transmit_frame (frame_t* frame)
{
    /* Insert the payload for transmission. CAN Classic has 2 words (8 bytes) for payload */
    for(uint8_t i = 0; i < MAX_MTU_WORDS; i++)
    {
        CAN0_MB -> Classic_MessageBuffer[TX_MB].payload[i] = frame -> payload[i];
    }

    /* Set the frame's destination ID */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].STD_ID = frame -> ID;

    /* Configure transmission message buffer. See "Message Buffer Structure" in RM */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].EDL =  0;   	/* No extended data length */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].BRS =  0;   	/* No bit-rate switch */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].ESI =  0;		/* No applies */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].SRR =  0;		/* No applies */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].IDE =  0;   	/* Standard ID */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].RTR =  0;		/* No remote request made */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].DLC = 0x8;  	/* 8 bytes of payload */
    CAN0_MB -> Classic_MessageBuffer[TX_MB].CODE = 0xC; 	/* After TX, the MB automatically returns to the INACTIVE state */

    /* After a successful transmission the interrupt flag of the corresponding message buffer is set */
    while(!(CAN0 -> CAN0_IFLAG1_b.BUF0I));

    /* Clear the flag previously polled (W1C register) */
    CAN0 -> CAN0_IFLAG1_b.BUF0I = 1;

    /* Return successful transmission request status */
    return Success;
}


/*!
* @brief Receive a single CAN frame
*
* @param [frame]  A reference to a frame for transmitting
*
* @return Success If a frame was read successfully
* @return Failure If at least an error occurred
*/
status_t FlexCAN_receive_frame (frame_t* frame)
{
    /* Default output and return values */
    status_t status = Failure;

    /* Check the RX message buffer */
    if(CAN0 -> CAN0_IFLAG1_b.BUF4TO1I)
    {
        /* Harvest the ID */
        frame -> ID = CAN0_MB -> Classic_MessageBuffer[RX_MB].STD_ID;

        /* Harvest the payload */
        for(uint8_t i = 0; i < MAX_MTU_WORDS; i++)
        {
            frame -> payload[i] = CAN0_MB -> Classic_MessageBuffer[RX_MB].payload[i];
        }

        /* Dummy read of the timer for unlocking the MB */
        (void)CAN0 -> CAN0_TIMER;

        /* Clear the flag previously polled (W1C register) */
        CAN0 -> CAN0_IFLAG1_b.BUF4TO1I = 1;

        /* Return success status code */
        status = Success;
    }
    return status;
}
