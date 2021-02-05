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
/*******************************************************************************
* Includes
*******************************************************************************/
#include "device_registers.h"
#include "clocks_and_modes.h"
#include "TOUCH1.h"
#include "TOUCH2.h"
#include "ADC.h"
#include <stdbool.h>

#define PTD0 (0)   	/* Port D0: FRDM EVB output to blue LED 	*/
#define PTD15 (15)	/* Port D15: FRDM EVB output to red LED 	*/
#define PTD16 (16) 	/* Port D16: FRDM EVB output to green LED 	*/

/*******************************************************************************
* Function prototypes
******************************************************************************/
bool TouchSense1 (void);
bool TouchSense2 (void);

/*****************************************************************************
*
* Function: void PORT_init (void)
* Description: Configure as output the corresponding GPIO's for the RGB led.
*
*****************************************************************************/
void PORT_init (void)
{
	/*!
	 * Pin number        | Function
	 * ----------------- |------------------
	 * PTD0              | GPIO [BLUE LED]
	 * PTD15             | GPIO [RED LED]
	 * PTD16			 | GPIO [GREEN LED]
	 */
    PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK;   /* Enable clock for PORTD */
    PORTD->PCR[PTD0]  = PORT_PCR_MUX(1);	/* Port D0: MUX = GPIO  */
    PORTD->PCR[PTD15] = PORT_PCR_MUX(1);  /* Port D15: MUX = GPIO */
    PORTD->PCR[PTD16] = PORT_PCR_MUX(1);  /* Port D16: MUX = GPIO */

    PTD ->PDDR |= 1<<PTD0		/* Port D0:  Data Direction = output */
    			|1<<PTD15	    /* Port D15: Data Direction = output */
				|1<<PTD16;    /* Port D16: Data Direction = output */

    /* Turn-off RGB led */
    PTD ->PSOR |= 1<<PTD0	/* Port D0:  Data Direction = output */
    			|1<<PTD15	/* Port D15: Data Direction = output */
				|1<<PTD16;  /* Port D16: Data Direction = output */
}

/*****************************************************************************
*
* Function: void WDOG_disable (void)
* Description: Disable the Watchdog timer.
*
*****************************************************************************/
void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

/*****************************************************************************
*
* Function: int main(void)
* Description: Sense touch pads
*
*****************************************************************************/
int main(void)
{
    WDOG_disable();        /* Disable WDOG												*/
    SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal 				*/
    SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC 				*/
    NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash*/
    PORT_init();		   /* Init  port clocks and gpio outputs 						*/
    ADC_init();            /* Init ADC resolution 12 bit								*/

    /* Enable clock for PORTC */
    PCC->PCCn[PCC_PORTC_INDEX ]|=PCC_PCCn_CGC_MASK;

    /* Endless loop */
    for(;;)
    {
    	/* If PAD1 is touched, Blue led turns on */
    	if(TouchSense1())
    	{
    		PTD ->PCOR |= 1 << PTD0;
    	}
    	else
    	{
    		PTD ->PSOR |= 1 << PTD0;
    	}

    	/* If PAD2 is touched, Green led turns on */
    	if(TouchSense2())
    	{
    		PTD ->PCOR |= 1 << PTD16;
    	}
    	else
    	{
    		PTD ->PSOR |= 1 << PTD16;
    	}
    }

    return 0;
}

/*****************************************************************************
*
* Function: bool TouchSense1 (void)
* Description: Sense pad1
*
*****************************************************************************/
bool TouchSense1 (void)
{
	bool Touched = false;
	uint16_t adcResult;
	uint32_t chargeDistributionPeriodTmp = ELEC1_DIST_CHARGE_DELAY;

	// Configure touch button (and EGS) electrodes floating
	ElectrodeFloat1();

	// Distribute Electrode and Cext charge
	ChargeDistribution1();

	// Delay to distribute charge
	while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

	// Start Cext voltage ADC conversion
	ADC_channel_convert(ELEC1_ADC_CHANNEL);

	// Redistribute Electrode and Cext charge
	ChargeRedistribution1();

	// Equivalent voltage digitalization
	// Wait for conversion complete flag
	while(ADC_conversion_complete()==0){}
	adcResult = ADC_channel_read();
	(void)adcResult;

	// Compare with the Touched reference.
	if(ADC_channel_read() < ELEC1_TOUCHED_LIMIT_VALUE)
	{
		Touched = true;
	}

	return Touched;
}

/*****************************************************************************
*
* Function: bool TouchSense2 (void)
* Description: Sense pad2
*
*****************************************************************************/
bool TouchSense2 (void)
{
	bool Touched = false;
	uint16_t adcResult;
	uint32_t chargeDistributionPeriodTmp = ELEC2_DIST_CHARGE_DELAY;

	// Configure touch button (and EGS) electrodes floating
	ElectrodeFloat2();

	// Distribute Electrode and Cext charge
	ChargeDistribution2();

	// Delay to distribute charge
	while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

	// Start Cext voltage ADC conversion
	ADC_channel_convert(ELEC2_ADC_CHANNEL);

	// Redistribute Electrode and Cext charge
	ChargeRedistribution2();

	// Equivalent voltage digitalization
	// Wait for conversion complete flag
	while(ADC_conversion_complete()==0){}
	adcResult = ADC_channel_read();
	(void)adcResult;

	// Compare with the Touched reference.
	if(ADC_channel_read() < ELEC2_TOUCHED_LIMIT_VALUE)
	{
		Touched = true;
	}

	return Touched;
}
