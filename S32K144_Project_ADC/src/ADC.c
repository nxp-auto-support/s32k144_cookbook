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

#include "ADC.h"

void ADC_init(void)
{
	/*!
	 * ADC0 Clocking:
	 * ===================================================
	 */
  PCC->PCCn[PCC_ADC0_INDEX] &=~ PCC_PCCn_CGC_MASK;  /* Disable clock to change PCS */
  PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_PCS(1);     /* PCS=1: Select SOSCDIV2 		*/
  PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;   /* Enable bus clock in ADC 	*/

	/*!
	 * ADC0 Initialization:
	 * ===================================================
	 */
  ADC0->SC1[0] |= ADC_SC1_ADCH_MASK;	/* ADCH=1F: Module is disabled for conversions	*/
                                  	  	/* AIEN=0: Interrupts are disabled 			*/
  ADC0->CFG1 |= ADC_CFG1_ADIV_MASK
		  	   |ADC_CFG1_MODE(1);	/* ADICLK=0: Input clk=ALTCLK1=SOSCDIV2 	*/
                                  	/* ADIV=0: Prescaler=1 					*/
                                  	/* MODE=1: 12-bit conversion 				*/

  ADC0->CFG2 = ADC_CFG2_SMPLTS(12);	/* SMPLTS=12(default): sample time is 13 ADC clks 	*/
  ADC0->SC2 = 0x00000000;         	/* ADTRG=0: SW trigger 							*/
                                  	/* ACFE,ACFGT,ACREN=0: Compare func disabled		*/
                                  	/* DMAEN=0: DMA disabled 							*/
                                    /* REFSEL=0: Voltage reference pins= VREFH, VREEFL */
  ADC0->SC3 = 0x00000000;       	/* CAL=0: Do not start calibration sequence 		*/
                                    /* ADCO=0: One conversion performed 				*/
                                    /* AVGE,AVGS=0: HW average function disabled 		*/
}
void ADC_init_HWTrigger(char Channel)  {

  PCC->PCCn[PCC_ADC0_INDEX] &=~ PCC_PCCn_CGC_MASK;  /* Disable clock to change PCS */
  PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_PCS(1);     /* PCS=1: Select SOSCDIV2 */
  PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;   /* Enable bus clock in ADC */

  ADC0->SC1[0] |= ADC_SC1_ADCH_MASK
              |ADC_SC1_AIEN_MASK;         /* ADCH=1F: Module is disabled for conversions*/
                                  /* AIEN=0: Interrupts are disabled */
  ADC0->CFG1 |= ADC_CFG1_ADIV_MASK
            |ADC_CFG1_MODE(1);/* ADICLK=0: Input clk=ALTCLK1=SOSCDIV2 */
                                  /* ADIV=0: Prescaler=1 */
                                  /* MODE=1: 12-bit conversion */
  ADC0->CFG2 = 0x00000000C;       /* SMPLTS=12(default): sample time is 13 ADC clks */
  ADC0->SC2 = ADC_SC2_ADTRG_MASK;         /* ADTRG=1: HW trigger */
                                  /* ACFE,ACFGT,ACREN=0: Compare func disabled */
                                  /* DMAEN=0: DMA disabled */
                                  /* REFSEL=0: Voltage reference pins= VREFH, VREEFL */
  ADC0->SC1[0] = ADC_SC1_ADCH(Channel)
      |ADC_SC1_AIEN_MASK;         /* ADCH=1F: Module is disabled for conversions*/
                                    /* AIEN=0: Interrupts are disabled */
  ADC0->SC3 = 0x00000000;         /* CAL=0: Do not start calibration sequence */
                                  /* ADCO=0: One conversion performed */
                                  /* AVGE,AVGS=0: HW average function disabled */
}

void convertAdcChan(uint16_t adcChan)
{
	/*!
	 * For SW trigger mode, SC1[0] is used
	 * ===================================================
	 */
  ADC0->SC1[0]&=~ADC_SC1_ADCH_MASK;	      /* Clear prior ADCH bits 	*/
  ADC0->SC1[0] = ADC_SC1_ADCH(adcChan);   /* Initiate Conversion		*/
}

uint8_t adc_complete(void)
{
  return ((ADC0->SC1[0] & ADC_SC1_COCO_MASK)>>ADC_SC1_COCO_SHIFT); 	/* Wait for completion */
}

uint32_t read_adc_chx(void)
{
  uint16_t adc_result=0;
  adc_result=ADC0->R[0];      					/* For SW trigger mode, R[0] is used 	*/
  return  (uint32_t) ((5000*adc_result)/0xFFF); /* Convert result to mv for 0-5V range */
}

