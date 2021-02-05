/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016 - 2019, NXP.
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
#ifndef TOUCH1_H_
#define TOUCH1_H_
/*******************************************************************************
* Includes
*******************************************************************************/
#include "device_registers.h"
/*******************************************************************************
* Touched limit
******************************************************************************/
#define ELEC1_TOUCHED_LIMIT_VALUE   (2000)
#define ELEC1_DIST_CHARGE_DELAY     (100)

/*******************************************************************************
* PCR defines
******************************************************************************/
// Configure pin as GPIO, clear ISF, ISF disabled, ALT1=GPIO,
// high drive strength, disable pulls, fast slew rate
#define PCR_GPIO1	0x01000140
// Configure pin as analog input
#define PCR_ANA1	0x00000000

/*******************************************************************************
* Electrode 1 (SW7)
******************************************************************************/
#define ELEC1
#define ELEC1_ADC             ADC0
#define ELEC1_ADC_CHANNEL     11
#define ELEC1_PORT            PORTC
#define ELEC1_GPIO            PTC
#define ELEC1_ELEC_GPIO_PIN   1
#define ELEC1_CEXT_GPIO_PIN   3
#define ELEC1_PORT_MASK       (1 << ELEC1_ELEC_GPIO_PIN) | (1 << ELEC1_CEXT_GPIO_PIN)

/*****************************************************************************
*
* Function: static inline void ChargeDistribution1 (void)
* Description: Charge distribution
*
*****************************************************************************/
static inline void ChargeDistribution1 (void)
{
   // Drive electrode GPIO low
	ELEC1_GPIO->PCOR = 1 << ELEC1_ELEC_GPIO_PIN;

    // Configure electrode pin as GPIO to precharge electrode
    ELEC1_PORT->PCR[ELEC1_ELEC_GPIO_PIN] = PCR_GPIO1;

    // Drive Cext GPIO high
    ELEC1_GPIO->PSOR = 1 << ELEC1_CEXT_GPIO_PIN;

    // Configure Cext pin as GPIO to precharge Cext
    ELEC1_PORT->PCR[ELEC1_CEXT_GPIO_PIN] = PCR_GPIO1;

    // Configure Electrode and Cext pins as outputs at the same time
    REG_WRITE32(&(ELEC1_GPIO->PDDR), ((REG_READ32(&(ELEC1_GPIO->PDDR))) | (ELEC1_PORT_MASK)));
}

/*****************************************************************************
*
* Function: static inline void ChargeRedistribution1 (void)
* Description: Charge redistribution
*
*****************************************************************************/
static inline void ChargeRedistribution1 (void)
{
	// Configure Electrode and Cext pins as inputs at the same time
	REG_WRITE32(&(ELEC1_GPIO->PDDR), ((REG_READ32(&(ELEC1_GPIO->PDDR))) & (~(ELEC1_PORT_MASK))));
}

/*****************************************************************************
*
* Function: static inline void ElectrodeFloat1 (void)
* Description: Configure electrode pins as an inputs (high impedance)
*
*****************************************************************************/
static inline void ElectrodeFloat1 (void)
{
    // Configure Electrode and Cext pins as inputs at the same time
	REG_WRITE32(&(ELEC1_GPIO->PDDR), ((REG_READ32(&(ELEC1_GPIO->PDDR))) & (~(ELEC1_PORT_MASK))));
}

#endif /* TOUCH1_H_ */
