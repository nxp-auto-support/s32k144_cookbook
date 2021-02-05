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
#ifndef TOUCH2_H_
#define TOUCH2_H_
/*******************************************************************************
* Includes
*******************************************************************************/
#include "device_registers.h"
/*******************************************************************************
* Touched limit
******************************************************************************/
#define ELEC2_TOUCHED_LIMIT_VALUE   (2000)
#define ELEC2_DIST_CHARGE_DELAY     (100)

/*******************************************************************************
* PCR defines
******************************************************************************/
// Configure pin as GPIO, clear ISF, ISF disabled, ALT1=GPIO,
// high drive strength, disable pulls, fast slew rate
#define PCR_GPIO2	0x01000140
// Configure pin as analog input
#define PCR_ANA2	0x00000000

/*******************************************************************************
* Electrode 2 (SW8)
******************************************************************************/
#define ELEC2
#define ELEC2_ADC             ADC0      // Modify: ADC module
#define ELEC2_ADC_CHANNEL     8         // Modify: Cext ADC channel
#define ELEC2_PORT            PORTC     // Modify: Electrode and Cext PORT
#define ELEC2_GPIO            PTC       // Modify: Electrode and Cext GPIO
#define ELEC2_ELEC_GPIO_PIN   2         // Modify: Electrode GPIO pin
#define ELEC2_CEXT_GPIO_PIN   0         // Modify: Cext GPIO pin
#define ELEC2_PORT_MASK       (1 << ELEC2_ELEC_GPIO_PIN) | (1 << ELEC2_CEXT_GPIO_PIN)

/*****************************************************************************
*
* Function: static inline void ChargeDistribution2 (void)
* Description: Charge distribution
*
*****************************************************************************/
static inline void ChargeDistribution2 (void)
{
   // Drive electrode GPIO low
	ELEC2_GPIO->PCOR = 1 << ELEC2_ELEC_GPIO_PIN;

    // Configure electrode pin as GPIO to precharge electrode
    ELEC2_PORT->PCR[ELEC2_ELEC_GPIO_PIN] = PCR_GPIO2;

    // Drive Cext GPIO high
    ELEC2_GPIO->PSOR = 1 << ELEC2_CEXT_GPIO_PIN;

    // Configure Cext pin as GPIO to precharge Cext
    ELEC2_PORT->PCR[ELEC2_CEXT_GPIO_PIN] = PCR_GPIO2;

    // Configure Electrode and Cext pins as outputs at the same time
    REG_WRITE32(&(ELEC2_GPIO->PDDR), ((REG_READ32(&(ELEC2_GPIO->PDDR))) | (ELEC2_PORT_MASK)));
}

/*****************************************************************************
*
* Function: static inline void ChargeRedistribution2 (void)
* Description: Charge redistribution
*
*****************************************************************************/
static inline void ChargeRedistribution2 (void)
{
	// Configure Electrode and Cext pins as inputs at the same time
	REG_WRITE32(&(ELEC2_GPIO->PDDR), ((REG_READ32(&(ELEC2_GPIO->PDDR))) & (~(ELEC2_PORT_MASK))));
}

/*****************************************************************************
*
* Function: static inline void ElectrodeFloat2 (void)
* Description: Configure electrode pins as an inputs (high impedance)
*
*****************************************************************************/
static inline void ElectrodeFloat2 (void)
{
    // Configure Electrode and Cext pins as inputs at the same time
	REG_WRITE32(&(ELEC2_GPIO->PDDR), ((REG_READ32(&(ELEC2_GPIO->PDDR))) & (~(ELEC2_PORT_MASK))));
}
#endif /* TOUCH2_H_ */
