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

/*!
 * Description:
 * ==========================================================================================
 * This example shows the LCD driver usage.
 * The communication between the MCU and the LCD driver is by SPI.
 * There are some functions made to show the well function of the driver.
 * This functions can be found in the TFT_LCD.h and STMPE610.h files.
 * The LCD driver used in this example is ILI9341 controller.
 * */

#include "device_registers.h"			/* include peripheral declarations */
#include "clocks_and_modes.h"
#include "TFT_LCD.h"
#include "STMPE610.h"
#include "LPSPI.h"

#define PTB2 (2)
#define PTB3 (3)
#define PTB4 (4)
#define PTB5 (5)
#define PTD14 (14)
#define PTD13 (13)

/*!
* @brief PORTn Initialization
*/
void PORT_init (void)
{
	/*!
	*           Pins Definitions
	* =====================================
	*
	*    Pin Number     |    Function
	* ----------------- |------------------
	* PTB2              | LPSPI0 [SCK]
	* PTB3			 	| LPSPI0 [SIN]
	* PTB4              | LPSPI0 [SOUT]
	* PTD14             | GPIO [LCD_Control_RS]
	* PTD13			 	| GPIO [Touch_CS]
	*/

	/* Port Configuration for LPSPI0 LCD Control */
	PCC -> PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;  	/* CGC=1: Clock enabled for PORTB */

	PORTB -> PCR[PTB2] |= PORT_PCR_MUX(3);     				/* MUX=3: Select LPSPI0_SCK  on PTB2 */
	PORTB -> PCR[PTB3] |= PORT_PCR_MUX(3);     				/* MUX=3: Select LPSPI0_SIN  on PTB3 */
	PORTB -> PCR[PTB4] |= PORT_PCR_MUX(3);     				/* MUX=3: Select LPSPI0_SOUT on PTB4 */

	PORTB -> PCR[PTB5] |= PORT_PCR_MUX(1);     				/* MUX=1: Select GPIO on PTB5 */
	PTB -> PSOR |= 1<<PTB5;                    				/* Assert LCD Chip Select */
	PTB -> PDDR |= 1<<PTB5;                    				/* Direction: PTB5 as output */

	/* Port Configuration for Other LCD Control (RS) */
	PCC -> PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;  	/* CGC=1: Clock enabled for PORTD */

	PORTD -> PCR[PTD14] |= PORT_PCR_MUX(1);     			/* MUX=1: Select GPIO on PTD14 */
	PTD -> PCOR |= 1<<PTD14; 								/* Clear PTD14 */
	PTD -> PDDR |= 1<<PTD14; 								/* PTD14 is an output */

	/* Port Configuration for Touch CS */
	PORTD -> PCR[PTD13] |= PORT_PCR_MUX(1);     			/* MUX=1: Select GPIO on PTD13 */
	PTD -> PSOR |= 1<<PTD13; 								/* Assert TOUCH Chip Select */
	PTD -> PDDR |= 1<<PTD13; 								/* PTD13 is also an output  */
}

void WDOG_disable (void)
{
	WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
	WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
	WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}

void Enable_Interrupt(uint8_t vector_number)
{
	S32_NVIC->ISER[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
	S32_NVIC->ICPR[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

int main(void)
{
	uint8_t x;
	uint8_t y;
	uint8_t z;

	/*!
	* Initialization:
	*/
	WDOG_disable();							/* Disable watchdog */
	SOSC_init_8MHz();						/* Initialize system oscilator for 8 MHz xtal */
	SPLL_init_160MHz();						/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz();					/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

	PORT_init();							/* Configure ports */
	LCD_display9341_init();					/* Initialize LCD Display */
    STMPE610_init();						/* Initialize STMPE610 */

    LCD_draw_string(80, 10, "Hello World!", WHITE, BLACK, 3);
    LCD_fill_triangle(10, 10, 10, 100, 50, 50, MAGENTA);
    LCD_draw_circle(270, 100, 25, CYAN);
    LCD_draw_square(20, 120, 100, RED);
    LCD_fill_rectangle(150, 100, 15, 80, YELLOW);
    LCD_draw_line(0, 0, TFT_HEIGHT, TFT_WIDTH, GREEN);

	for(;;)
	{
		if(STMPE610_touched())				/* Evaluate the touch-pad was activated */
		{
			STMPE610_get_XYZ(&x, &y, &z);	/* Get the (x,y,z) coordinates of the touched surface */
		}
	}
}
