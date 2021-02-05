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

#include "device_registers.h"	/* include peripheral declarations S32K144 */
#include "STMPE610.h"
#include "LPSPI.h"

/* Private Function Prototypes */
void STMPE610_write_register 	(uint8_t reg, uint8_t val);
uint8_t STMPE610_read_register 	(uint8_t reg);
uint8_t STMPE610_check_version 	(void);

/*!
* @brief Write a byte through the pre-configured SPI to the controller.
*
* @param[uint8_t reg] Register.
* @param[uint8_t val] 8-bit data.
*/
void STMPE610_write_register (uint8_t reg, uint8_t val)
{
	/* Write a register into the STMPE610 controller */
	STMPE610_CS_low();									/* CS is active */
	LPSPI0_write_8bits(reg);
	LPSPI0_write_8bits(val);
	STMPE610_CS_high();								/* CS is inactive */
}

/*!
* @brief Read a byte through the pre-configured SPI from the controller.
*
* @param[uint8_t reg] Register.
*
* @return[uint8_t temp] 8-bit data from the selected register.
*/
uint8_t STMPE610_read_register (uint8_t reg)
{
	uint8_t temp;

	/* Read a register from the STMPE610 controller */
	STMPE610_CS_low();									/* CS is active */
	LPSPI0_write_8bits(0x80 | reg);
	LPSPI0_write_8bits(0x00);
    temp = LPSPI0_write_8bits(STMPE_NOP);
    STMPE610_CS_high();								/* CS is inactive */

    return temp;
}

/*!
* @brief Verify the version of the controller.
*
* @return False (0) if the LSB/MSB byte does not match with STMPE610 version.
* @return True (1) if the version was successfully authenticated.
*/
uint8_t STMPE610_check_version (void)
{
	/* If LSB byte does not match with STMPE610 version, return false */
	if (STMPE_MSB_VERSIONSUPPORTED == STMPE610_read_register(0x00));
	else
	{
		return 0;
	}

	/* If MSB byte does not match with STMPE610 version, return false */
	if (STMPE_LSB_VERSIONSUPPORTED == STMPE610_read_register(0x01));
	else
	{
		return 0;
	}

	/* Version was successfully authenticated */
	return 1;
}

/* Public Function Prototypes */

/*!
* @brief Initialize STMPE610.
*
* @return False (0) if version was not successfully authenticated.
* @return True (1) if the initialization was successfully.
*/
uint8_t STMPE610_init (void)
{
	uint8_t i;

    LPSPI0_init_master();

    /* Verify STMPE610 controller */
	if (!STMPE610_check_version())
	{
		return 0;
	}

	/* Reset STMPE610 */
	STMPE610_write_register(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);

	/* Flush all registers after software reset */
	for(i = 0; i < 65; i++)
	{
		STMPE610_read_register(i);
	}

    /* STMPE610 initialization */
	STMPE610_write_register(STMPE_SYS_CTRL2, 0x0);
	STMPE610_write_register(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN);
	STMPE610_write_register(STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
	STMPE610_write_register(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT | (0x6 << 4));
	STMPE610_write_register(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
	STMPE610_write_register(STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE |	STMPE_TSC_CFG_DELAY_1MS | STMPE_TSC_CFG_SETTLE_5MS);
	STMPE610_write_register(STMPE_TSC_FRACTION_Z, 0x6);
	STMPE610_write_register(STMPE_FIFO_TH, 1);
	STMPE610_write_register(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
	STMPE610_write_register(STMPE_FIFO_STA, 0);
	STMPE610_write_register(STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
	STMPE610_write_register(STMPE_INT_STA, 0xFF);
	STMPE610_write_register(STMPE_INT_CTRL, STMPE_INT_CTRL_POL_HIGH | STMPE_INT_CTRL_ENABLE);

	return 1;
}

/*!
* @brief Evaluate the status of the touch-pad.
*
* @return True if the screen was touched.
*/
uint8_t STMPE610_touched (void)
{
    return (STMPE610_read_register(STMPE_TSC_CTRL) & 0x80);
}

/*!
* @brief Get the (x,y,z) coordinates of the touched surface.
*
* @param[uint8_t *x] Pointer to the X axis.
* @param[uint8_t *y] Pointer to the Y axis.
* @param[uint8_t *z] Pointer to the Z axis.
*/
void STMPE610_get_XYZ (uint8_t *x, uint8_t *y, uint8_t *z)
{
	uint8_t temp_data[4];
	uint8_t i;

	for (i = 0; i < 4; i++)
	{
		temp_data[i] = STMPE610_read_register(0xD7);
	}

	*x = temp_data[0];
	*x <<= 4;
	*x |= (temp_data[1] >> 4);
	*y = temp_data[1] & 0x0F;
	*y <<= 8;
	*y |= temp_data[2];
	*z = temp_data[3];
}
