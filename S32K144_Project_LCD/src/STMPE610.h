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

#ifndef STMPE610_H_
#define STMPE610_H_

/* Defines and Variables */
/* Reset Control */
#define STMPE_SYS_CTRL1 			0x03
#define STMPE_SYS_CTRL1_RESET 		0x02

/* Clock Control */
#define STMPE_SYS_CTRL2 			0x04

/* Touch-Screen Controller Setup */
#define STMPE_TSC_CTRL 				0x40
#define STMPE_TSC_CTRL_EN 			0x01
#define STMPE_TSC_CTRL_XYZ 			0x00
#define STMPE_TSC_CTRL_XY 			0x02

/* Interrupt Control */
#define STMPE_INT_CTRL 				0x09
#define STMPE_INT_CTRL_POL_HIGH 	0x04
#define STMPE_INT_CTRL_POL_LOW 		0x00
#define STMPE_INT_CTRL_EDGE 		0x02
#define STMPE_INT_CTRL_LEVEL 		0x00
#define STMPE_INT_CTRL_ENABLE 		0x01
#define STMPE_INT_CTRL_DISABLE 		0x00

/* Interrupt Enable */
#define STMPE_INT_EN 				0x0A
#define STMPE_INT_EN_TOUCHDET 		0x01
#define STMPE_INT_EN_FIFOTH 		0x02
#define STMPE_INT_EN_FIFOOF 		0x04
#define STMPE_INT_EN_FIFOFULL 		0x08
#define STMPE_INT_EN_FIFOEMPTY		0x10
#define STMPE_INT_EN_ADC 			0x40
#define STMPE_INT_EN_GPIO 			0x80

/* Interrupt Status */
#define STMPE_INT_STA 				0x0B
#define STMPE_INT_STA_TOUCHDET 		0x01

/* ADC Control */
#define STMPE_ADC_CTRL1 			0x20
#define STMPE_ADC_CTRL1_12BIT 		0x08
#define STMPE_ADC_CTRL1_10BIT 		0x00

/* ADC Control */
#define STMPE_ADC_CTRL2 			0x21
#define STMPE_ADC_CTRL2_1_625MHZ 	0x00
#define STMPE_ADC_CTRL2_3_25MHZ 	0x01
#define STMPE_ADC_CTRL2_6_5MHZ 		0x02

/* Touch-Screen Controller Configuration */
#define STMPE_TSC_CFG 				0x41
#define STMPE_TSC_CFG_1SAMPLE 		0x00
#define STMPE_TSC_CFG_2SAMPLE 		0x40
#define STMPE_TSC_CFG_4SAMPLE 		0x80
#define STMPE_TSC_CFG_8SAMPLE 		0xC0
#define STMPE_TSC_CFG_DELAY_10US 	0x00
#define STMPE_TSC_CFG_DELAY_50US 	0x08
#define STMPE_TSC_CFG_DELAY_100US 	0x10
#define STMPE_TSC_CFG_DELAY_500US 	0x18
#define STMPE_TSC_CFG_DELAY_1MS 	0x20
#define STMPE_TSC_CFG_DELAY_5MS 	0x28
#define STMPE_TSC_CFG_DELAY_10MS 	0x30
#define STMPE_TSC_CFG_DELAY_50MS 	0x38
#define STMPE_TSC_CFG_SETTLE_10US 	0x00
#define STMPE_TSC_CFG_SETTLE_100US 	0x01
#define STMPE_TSC_CFG_SETTLE_500US 	0x02
#define STMPE_TSC_CFG_SETTLE_1MS 	0x03
#define STMPE_TSC_CFG_SETTLE_5MS 	0x04
#define STMPE_TSC_CFG_SETTLE_10MS 	0x05
#define STMPE_TSC_CFG_SETTLE_50MS 	0x06
#define STMPE_TSC_CFG_SETTLE_100MS 	0x07

/* FIFO Level to Generate Interrupt */
#define STMPE_FIFO_TH 				0x4A

/* Current Filled Level of FIFO */
#define STMPE_FIFO_SIZE 			0x4C

/* Current Status of FIFO */
#define STMPE_FIFO_STA 				0x4B
#define STMPE_FIFO_STA_RESET 		0x01
#define STMPE_FIFO_STA_OFLOW 		0x80
#define STMPE_FIFO_STA_FULL 		0x40
#define STMPE_FIFO_STA_EMPTY 		0x20
#define STMPE_FIFO_STA_THTRIG 		0x10

/* Touch-Screen Controller Driver */
#define STMPE_TSC_I_DRIVE 			0x58
#define STMPE_TSC_I_DRIVE_20MA 		0x00
#define STMPE_TSC_I_DRIVE_50MA 		0x01

/* Data Port for TSC Data Address */
#define STMPE_TSC_DATA_X 			0x4D
#define STMPE_TSC_DATA_Y 			0x4F
#define STMPE_TSC_FRACTION_Z 		0x56

/* SPI Read */
#define STMPE_NOP					0xFF
#define STMPE_MSB_VERSIONSUPPORTED 	0x08
#define STMPE_LSB_VERSIONSUPPORTED 	0x11

/* Public Function Prototypes */
uint8_t STMPE610_init 		(void);
uint8_t STMPE610_touched 	(void);
void STMPE610_get_XYZ 		(uint8_t *x, uint8_t *y, uint8_t *z);

#endif /* STMPE610_H_ */
