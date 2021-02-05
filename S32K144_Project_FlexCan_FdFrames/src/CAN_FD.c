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

#include "CAN_FD.h"
#include "register_bit_fields.h"
#include "stdint.h"

#define __IOM volatile 							/* The compiler won't optimize this macro */

/*!
* @brief Bit field declaration for the transmission and reception message buffers. See "Message Buffer Structure" in RM.
* 		 NOTE: Since this example manages CAN-FD (8 bytes for header and 64 bytes for payload)
* 		 there are 7 Message Buffers available. The FIFO is not enabled.
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
      __IOM uint32_t EXT_ID     : 29;
      __IOM uint32_t PRIO       : 3;
      __IOM uint32_t payload[16];				/* 64 bytes (16 words) for payload */
    } FD_MessageBuffer[7];						/* 7 MBs available */
} CAN0_MB_t;

/*!
* @brief Type casting of the structure declared above to the corresponding CAN0 memory area.
*        Refer to the header file "register_bit_fields.h" located inside the project's
*        "include" folder for further detail.
* 		 0x80 is the corresponding offset where the message buffer's memory area is located.
*/
#define CAN0_MB ((CAN0_MB_t*)(CAN0_BASE + 0x80))


/*!
* @brief Structure for the CAN bit timings in nominal and data phases. See "Protocol Timing" in RM (FlexCAN Chapter)
*/
typedef struct
{
	uint8_t EPRESDIV;
    uint8_t EPROPSEG;
    uint8_t EPSEG1;
    uint8_t EPSEG2;
    uint8_t ERJW;
    uint8_t FPRESDIV;
    uint8_t FPROPSEG;
    uint8_t FPSEG1;
    uint8_t FPSEG2;
    uint8_t FRJW;
} CAN_bit_timings_t;


/* CAN bit timings for nominal phase at 1 Mbit/s with 80 time quantas
 * and data phase at 4 Mbit/s with 20 time quantas */
CAN_bit_timings_t timings =
{
	.EPRESDIV = 0,
    .EPROPSEG = 46,
    .EPSEG1 = 18,
    .EPSEG2 = 12,
    .ERJW = 12,
    .FPRESDIV = 0,
    .FPROPSEG = 7,
    .FPSEG1 = 6,
    .FPSEG2 = 4,
    .FRJW = 4

	/* NOMINAL PHASE */
	/* (EPRESDIV + 1) * (EPROPSEG + EPSEG1 + EPSEG2 + 4) = 80 quantas */
	/* Extended Resynchronization Jump Width = ERJW + 1 = 13 */
	/* Bit Timing = FlaxCan CLK / time quantas = 80 MHz / 80 = 1 Mbit/s */

	/* DATA PHASE */
	/* (FPRESDIV + 1) * (FPROPSEG + FPSEG1 + FPSEG2 + 3) = 20 quantas */
	/* Fast Resynchronization Jump Width = FRJW + 1 = 5 */
	/* Bit Timing = FlaxCan CLK / time quantas = 80 MHz / 20 = 4 Mbit/s */
};


/*!
* @brief Enum for the index of the 2 Message Buffers (MB) used, 0th for TX and the 1st for RX.
* 		 There are a total of 7 MBs available, however in this project only 2 MBs are used.
*/
typedef enum
{
    TX_MB = 0,
    RX_MB = 1
} MB_index_Enum;


/*!
* @brief FlexCAN Initialization for FD Frames transmission and reception at 4 Mbit/s and 1 Mbit/s in data and nominal phases respectively
*
* @return Success If the peripheral was started without errors
*/
status_t FlexCAN_init_FD (void)
{
    PCC -> PCC_FlexCAN0_b.CGC   = PCC_PCC_FlexCAN0_CGC_1; 	/* FlexCAN0 clock gating */

    /* Set Normal RUN clock mode for feeding SYS_CLK @ 80 MHz to FlexCAN */
    CAN0 -> CAN0_MCR_b.MDIS     = CAN0_MCR_MDIS_1;        	/* Disable FlexCAN module for clock source selection */
    CAN0 -> CAN0_CTRL1_b.CLKSRC = CAN0_CTRL1_CLKSRC_1;    	/* Select SYS_CLK as source (80 MHz) */
    CAN0 -> CAN0_MCR_b.MDIS     = CAN0_MCR_MDIS_0;        	/* Enable FlexCAN peripheral */
    CAN0 -> CAN0_MCR_b.HALT     = CAN0_MCR_HALT_1;        	/* Request freeze mode entry */
    CAN0 -> CAN0_MCR_b.FRZ      = CAN0_MCR_FRZ_1;			/* Enter in freeze mode */

    /* Block for freeze mode entry */
    while(!(CAN0 -> CAN0_MCR_b.FRZACK));

    /* Enable CAN-FD feature in ISO 11898-1 compliance */
    CAN0 -> CAN0_MCR_b.FDEN = CAN0_MCR_FDEN_1;
    CAN0 -> CAN0_CTRL2_b.ISOCANFDEN = CAN0_CTRL2_ISOCANFDEN_1;

    /* CAN Bit Timing (CBT) configuration for a nominal phase of 1 Mbit/s with 80 time quantas, in accordance with Bosch 2012 specification, sample point at 83.75% */
    CAN0 -> CAN0_CBT_b.BTF 		= CAN0_CBT_BTF_1;
    CAN0 -> CAN0_CBT_b.EPRESDIV = timings.EPRESDIV;
    CAN0 -> CAN0_CBT_b.EPROPSEG = timings.EPROPSEG;
    CAN0 -> CAN0_CBT_b.EPSEG1   = timings.EPSEG1;
    CAN0 -> CAN0_CBT_b.EPSEG2   = timings.EPSEG2;
    CAN0 -> CAN0_CBT_b.ERJW     = timings.ERJW;

    /* CAN-FD Bit Timing (FDCBT) for a data phase of 4 Mbit/s with 20 time quantas, in accordance with Bosch 2012 specification, sample point at 75% */
    CAN0 -> CAN0_FDCBT_b.FPRESDIV = timings.FPRESDIV;
    CAN0 -> CAN0_FDCBT_b.FPROPSEG = timings.FPROPSEG;
    CAN0 -> CAN0_FDCBT_b.FPSEG1   = timings.FPSEG1;
    CAN0 -> CAN0_FDCBT_b.FPSEG2   = timings.EPSEG2;
    CAN0 -> CAN0_FDCBT_b.FRJW     = timings.FRJW;

    CAN0 -> CAN0_FDCTRL_b.FDRATE = CAN0_FDCTRL_FDRATE_1;  	/* Enable bit rate switch in data phase of frame */
    CAN0 -> CAN0_FDCTRL_b.TDCEN  = CAN0_FDCTRL_TDCEN_1;   	/* Enable transceiver delay compensation */
    CAN0 -> CAN0_FDCTRL_b.TDCOFF = 5;                     	/* Setup 5 cycles for data phase sampling delay */
    CAN0 -> CAN0_FDCTRL_b.MBDSR0 = CAN0_FDCTRL_MBDSR0_11; 	/* Setup 64 bytes per message buffer (7 MB's) */

    CAN0 -> CAN0_MCR_b.MAXMB  = 1;                 			/* Maximum number of MB's as 2 */
    CAN0 -> CAN0_MCR_b.SRXDIS = CAN0_MCR_SRXDIS_1; 			/* Disable self-reception of frames if ID matches */
    CAN0 -> CAN0_MCR_b.IRMQ   = CAN0_MCR_IRMQ_1;   			/* Enable individual message buffer ID masking */

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
* @param [uint32_t id] Extended ID
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

    /* An "All-care" bits mask 0x1FFFFFFF is used for the 29 bits of CAN-FD IDs */
    CAN0 -> CAN0_RXIMR0_b.MI = 0x1FFFFFFF;

    /* Configure reception message buffer. See "Message Buffer Structure" in RM */
    CAN0_MB -> FD_MessageBuffer[RX_MB].EDL =  1;			/* Extended data length */
    CAN0_MB -> FD_MessageBuffer[RX_MB].BRS =  1;			/* Bit-rate switch */
    CAN0_MB -> FD_MessageBuffer[RX_MB].ESI =  0;			/* No applies */
    CAN0_MB -> FD_MessageBuffer[RX_MB].CODE = 4;			/* When a frame is received successfully, this field is automatically updated to FULL */
    CAN0_MB -> FD_MessageBuffer[RX_MB].SRR =  0;			/* No applies */
    CAN0_MB -> FD_MessageBuffer[RX_MB].IDE =  1;			/* Extended ID */
    CAN0_MB -> FD_MessageBuffer[RX_MB].RTR =  0;			/* No remote request made */
    CAN0_MB -> FD_MessageBuffer[RX_MB].DLC = 0xF;  			/* 64 bytes of payload */

    /* Configure the ID */
    CAN0_MB -> FD_MessageBuffer[RX_MB].EXT_ID = id;

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
status_t FlexCAN_transmit_frame (fd_frame_t* frame)
{
	/* Insert the payload for transmission. CAN-FD has 16 words (64 bytes) for payload */
    for(uint8_t i = 0; i < MAX_MTU_WORDS; i++)
    {
        CAN0_MB -> FD_MessageBuffer[TX_MB].payload[i] = frame -> payload[i];
    }

    /* Set the frame's destination ID */
    CAN0_MB -> FD_MessageBuffer[TX_MB].EXT_ID = frame -> ID;

    /* Configure transmission message buffer. See "Message Buffer Structure" in RM */
    CAN0_MB -> FD_MessageBuffer[TX_MB].EDL =  1;			/* Extended data length */
    CAN0_MB -> FD_MessageBuffer[TX_MB].BRS =  1;			/* Bit-rate switch */
    CAN0_MB -> FD_MessageBuffer[TX_MB].ESI =  0;			/* No applies */
    CAN0_MB -> FD_MessageBuffer[TX_MB].SRR =  0;			/* No applies */
    CAN0_MB -> FD_MessageBuffer[TX_MB].IDE =  1;			/* Extended ID */
    CAN0_MB -> FD_MessageBuffer[TX_MB].RTR =  0;			/* No remote request made */
    CAN0_MB -> FD_MessageBuffer[TX_MB].DLC = 0xF;			/* 64 bytes of payload */
    CAN0_MB -> FD_MessageBuffer[TX_MB].CODE = 0xC; 			/* After TX, the MB automatically returns to the INACTIVE state */

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
status_t FlexCAN_receive_frame (fd_frame_t* frame)
{
    /* Default output and return values */
    status_t status = Failure;

    /* Check the RX message buffer */
    if(CAN0 -> CAN0_IFLAG1_b.BUF4TO1I )
    {
        /* Harvest the ID */
        frame -> ID = CAN0_MB -> FD_MessageBuffer[RX_MB].EXT_ID;

        /* Harvest the payload */
        for(uint8_t i = 0; i < MAX_MTU_WORDS; i++)
        {
            frame -> payload[i] = CAN0_MB -> FD_MessageBuffer[RX_MB].payload[i];
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
