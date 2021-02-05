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

#include "device_registers.h"           /* include peripheral declarations */
#include "ADC.h"

/*!
* @brief ADC Initialization for SW trigger without calibration
*/
void ADC_init (void)
{
	/* ADC0 Clocking */
	PCC -> PCCn[PCC_ADC0_INDEX] &= ~PCC_PCCn_CGC_MASK;  		/* Disable clock to change PCS */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_PCS(1);     		/* PCS = 1 Select SOSCDIV2 */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;   		/* Enable bus clock in ADC */

	/* ADC0 Initialization */
	ADC0 -> SC1[0] |= ADC_SC1_ADCH_MASK;						/* ADCH = 1F Module is disabled for conversions	*/
                                  	  							/* AIEN = 0 Interrupts are disabled */

	ADC0 -> CFG1 |= ADC_CFG1_ADIV_MASK							/* ADIV = 0 Prescaler = 1 */
		  	     |  ADC_CFG1_MODE(1);							/* MODE = 1 12-bit conversion */
																/* ADICLK = 0 Input CLK = ALTCLK1 = SOSCDIV2 */

	ADC0 -> CFG2 = ADC_CFG2_SMPLTS(12);							/* SMPLTS = 12(default): sample time is 13 ADC clks */

	ADC0 -> SC2 = 0x00000000;         							/* ADTRG = 0 SW trigger */
                                  								/* ACFE, ACFGT, ACREN = 0 Compare functionality disabled */
                                  								/* DMAEN = 0 DMA disabled */
																/* REFSEL = 0 Voltage reference pins= VREFH, VREEFL */

	ADC0 -> SC3 = 0x00000000;       							/* CAL = 0 Do not start calibration sequence */
                                    							/* ADCO = 0 One conversion performed */
                                    							/* AVGE, AVGS = 0 HW average function disabled 	*/
}

/*!
* @brief ADC Initialization for HW trigger without calibration
*/
void ADC_HW_trigger_init (int8_t adc_channel)
{
	/* ADC0 Clocking */
	PCC -> PCCn[PCC_ADC0_INDEX] &= ~PCC_PCCn_CGC_MASK;  		/* Disable clock to change PCS */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_PCS(1);     		/* PCS = 1 Select SOSCDIV2 */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;   		/* Enable bus clock in ADC */

	ADC0 -> SC1[0] |= ADC_SC1_ADCH_MASK 						/* ADCH = 1F Module is disabled for conversions */
                   |  ADC_SC1_AIEN_MASK;        				/* AIEN = 1 Interrupts are enabled */

	ADC0 -> CFG1 |= ADC_CFG1_ADIV_MASK 							/* ADIV = 0 Prescaler = 1 */
                 |  ADC_CFG1_MODE(1); 							/* MODE = 1 12-bit conversion */
																/* ADICLK = 0 Input CLK = ALTCLK1 = SOSCDIV2 */

	ADC0 -> CFG2 = 0x00000000C;       							/* SMPLTS = 12(default): sample time is 13 ADC clks */

	ADC0 -> SC2 = ADC_SC2_ADTRG_MASK;         					/* ADTRG = 1 HW trigger */
                                  	  	  	  	  	  	  	  	/* ACFE, ACFGT, ACREN = 0: Compare functionality disabled */
                                  	  	  	  	  	  	  	  	/* DMAEN = 0 DMA disabled */
                                  	  	  	  	  	  	  	  	/* REFSEL = 0 Voltage reference pins= VREFH, VREEFL */

	ADC0 -> SC1[0] = ADC_SC1_ADCH(adc_channel)					/* ADCH = 1F Module is disabled for conversions*/
	               | ADC_SC1_AIEN_MASK;  			 			/* AIEN = 1 Interrupts are enabled */


	ADC0 -> SC3 = 0x00000000;         							/* CAL = 0 Do not start calibration sequence */
                                  	  	  	  	  	  	  	  	/* ADCO = 0 One conversion performed */
                                  	  	  	  	  	  	  	  	/* AVGE, AVGS = 0 HW average function disabled */
}

/*!
* @brief Initiate ADC conversion in the desired channel.
*
* @param[uint16_t adc_channel] ADC Channel.
*/
void ADC_channel_convert (uint16_t adc_channel)
{
	/* For SW trigger mode, SC1[0] is used */
	ADC0 -> SC1[0] &= ~ADC_SC1_ADCH_MASK;	      				/* Clear prior ADCH bits */
	ADC0 -> SC1[0] =  ADC_SC1_ADCH(adc_channel);   				/* Initiate Conversion */
}

/*!
* @brief Check the conversion complete flag [COCO].
*
* @return [COCO] Conversion Flag Status.
*/
uint8_t ADC_conversion_complete (void)
{
	/* Conversion Flag Status [COCO] */
	return ((ADC0 -> SC1[0] & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT);
}

/*!
* @brief Read ADC channel and convert result to mV for 0-5 V range.
*
* @return[uint32_t] ADC result in mV.
*/
uint32_t ADC_channel_read (void)
{
	uint16_t adc_raw_result = 0;
	adc_raw_result = ADC0 -> R[0];      						/* For SW trigger mode, R[0] is used */
	return (uint32_t)((5000 * adc_raw_result) / 0xFFF); 		/* Convert result to mV for 0-5 V range */
}

/*!
* @brief ADC Initialization for SW trigger with calibration
*
* @param[uint16_t gain] UG User Gain.
* @param[uint16_t gain] USR_OFS User Offset.
*/
void ADC_calibration_init (int16_t gain, int16_t offset)
{
	/* ADC0 Clocking */
	PCC -> PCCn[PCC_ADC0_INDEX] &= ~PCC_PCCn_CGC_MASK;  		/* Disable clock to change PCS */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_PCS(1);     		/* PCS = 1 Select SOSCDIV2 */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;   		/* Enable bus clock in ADC */

	/* ADC0 Calibration */
	ADC0 -> CFG1 |= ADC_CFG1_ADIV(2) 							/* ADIV = 0 Prescaler = 1 */
				 |  ADC_CFG1_MODE(1);							/* MODE = 1 12-bit conversion */
																/* ADICLK = 0 Input CLK = ALTCLK1 = SOSCDIV2 */

	ADC0 -> SC3 |= ADC_SC3_AVGE_MASK							/* AVGE = 1 Enable hardware average */
	  			| ADC_SC3_AVGS(3);								/* AVGS = 11b 32 samples averaged */

	/* Initialization for the Internal ADC0 Calibration Registers */
	ADC0 -> CLPS = 0x00u;
	ADC0 -> CLP3 = 0x00u;
	ADC0 -> CLP2 = 0x00u;
	ADC0 -> CLP1 = 0x00u;
	ADC0 -> CLP0 = 0x00u;
	ADC0 -> CLPX = 0x00u;
	ADC0 -> CLP9 = 0x00u;
	ADC0 -> USR_OFS = offset;
	ADC0 -> UG = gain;

	ADC0 -> SC3 |= ADC_SC3_CAL_MASK;							/* CAL = 1 Start calibration sequence */

	while(ADC_conversion_complete() == 0);          			/* Wait for conversion complete flag */

	/* ADC0 Initialization */
	ADC0 -> SC1[0] |= ADC_SC1_ADCH_MASK; 						/* ADCH = 1F Module is disabled for conversions */
																/* AIEN = 0 Interrupts are disabled */

	ADC0 -> CFG1 |= ADC_CFG1_ADIV_MASK 							/* ADIV = 0 Prescaler = 1 */
                 |  ADC_CFG1_MODE(1); 							/* MODE = 1 12-bit conversion */
																/* ADICLK = 0 Input CLK = ALTCLK1 = SOSCDIV2 */

	ADC0 -> CFG2 = ADC_CFG2_SMPLTS(12);       					/* SMPLTS = 12(default): sample time is 13 ADC clks */

	ADC0 -> SC2 = 0x00000000;         							/* ADTRG = 0 SW trigger */
																/* ACFE, ACFGT, ACREN = 0 Compare functionality disabled */
	                                  							/* DMAEN = 0 DMA disabled */
																/* REFSEL = 0 Voltage reference pins= VREFH, VREEFL */


	ADC0 -> SC3 = 0x00000000;         							/* CAL = 0 Do not start calibration sequence */
                                  	  	  	  	  	  	  	  	/* ADCO = 0 One conversion performed */
                                  	  	  	  	  	  	  	  	/* AVGE, AVGS = 0 HW average function disabled */
}

/*! Configuration of 4 channels from the ADC0, those channels are
 * 	trigger from the PDB, the results are saved with the DMA.
 * 		ADC0->SC1[4] Pot
 * 		ADC0->SC1[5] Pot
 * 		ADC0->SC1[6] 3.3V (You must connect any voltage at PTB0)
 * 		ADC0->SC1[7] 3.3V (You must connect any voltage at PTB0)
 *
 * 		@param [uint8_t PotCh] Enters the Pot Channel of the EVB
 */
void ADC_Config(uint8_t Pot_Ch){
	ADC0->SC1[4] = ADC_SC1_ADCH_MASK;	/* Channel 4 is disabled */
	ADC0->SC1[5] = ADC_SC1_ADCH_MASK;	/* Channel 5 is disabled */
	ADC0->SC1[6] = ADC_SC1_ADCH_MASK;	/* Channel 6 is disabled */
	ADC0->SC1[7] = ADC_SC1_ADCH_MASK;	/* Channel 7 is disabled */

	ADC0->CFG1 = ADC_CFG1_ADIV(0)|	/* Divide ratio = 1 */
				 ADC_CFG1_MODE(1);	/*	12-bit conversion */

	ADC0->CFG2 = ADC_CFG2_SMPLTS(9);	/* sample time is 10 ADC clks */

	ADC0->SC2 = ADC_SC2_ADTRG(1)|	/* ADTRG = 1: HW trigger */
				ADC_SC2_DMAEN_MASK; /* DMA interrupt enable */

	ADC0->SC1[4] = ADC_SC1_ADCH(Pot_Ch);	/* External channel as input (Pot) */
	ADC0->SC1[5] = ADC_SC1_ADCH(Pot_Ch);	/* External channel as input (Pot) */
	ADC0->SC1[6] = ADC_SC1_ADCH(4);		/* External channel 4 as input (voltage at PTB0) */
	ADC0->SC1[7] = ADC_SC1_ADCH(4);		/* External channel 4 as input (voltage at PTB0) */
	ADC0->SC3 = 0x00000000; 			/* Disable any configuration enabled of the calibration */
}

/*!
 * ADC Configuration to work using HW Trigger (PDB) and to generate a DMA request
 * when COCO is set.
 */
void ADC_FlexScan_Config(void){
	PCC -> PCCn[PCC_ADC0_INDEX] &= ~PCC_PCCn_CGC_MASK;  		/* Disable clock to change PCS */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_PCS(1);     		/* PCS = 1 Select SOSCDIV2 */
	PCC -> PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;   		/* Enable bus clock in ADC */

	ADC0 -> CFG1 |= ADC_CFG1_ADIV_MASK 							/* ADIV = 0 Prescaler = 1 */
                 |  ADC_CFG1_MODE(1); 							/* MODE = 1 12-bit conversion */
																/* ADICLK = 0 Input CLK = ALTCLK1 = SOSCDIV2 */

	ADC0 -> CFG2 = 0x00000000C;       							/* SMPLTS = 12(default): sample time is 13 ADC clks */

	ADC0 -> SC2 |= ADC_SC2_ADTRG_MASK |							/* ADTRG = 1 HW trigger */
				  ADC_SC2_DMAEN_MASK;         					/* ACFE, ACFGT, ACREN = 0: Compare functionality disabled */
                                  	  	  	  	  	  	  	  	/* DMAEN = 1 DMA disabled */
                                  	  	  	  	  	  	  	  	/* REFSEL = 0 Voltage reference pins= VREFH, VREEFL */

	ADC0 -> SC1[0] = ADC_SC1_ADCH(12);					/* ADCH = 12 to start measuring the Potentiometer,*/
}
