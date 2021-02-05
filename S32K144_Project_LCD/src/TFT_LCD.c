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

#include <stdlib.h>
#include "device_registers.h"	/* include peripheral declarations */
#include "TFT_LCD.h"
#include "LPSPI.h"

/* Set Width and Height */
#if (TFT_ORIGIN & 0x20)
	static uint16_t width = TFT_HEIGHT;
	static uint16_t height = TFT_WIDTH;
#else
	static uint16_t width = TFT_WIDTH;
	static uint16_t height = TFT_HEIGHT;
#endif

/* Defines and Variables */
#define swap(a, b) { int16_t t = a; a = b; b = t; }
static const uint8_t font[] = TFT_FONT;

/* Private Function Prototypes */
void TFT_LCD_delay           	(uint32_t wait);
void TFT_LCD_transmit_8bits  	(uint8_t val);
void TFT_LCD_write_8data     	(uint8_t data);
void TFT_LCD_write_16data    	(uint16_t data);
void TFT_LCD_write_8command  	(uint8_t cmd);
void TFT_LCD_write_16command 	(uint16_t cmd);
void TFT_LCD_write_16register 	(uint16_t addr, uint16_t data);
void TFT_LCD_set_address_window (int16_t x1, int16_t y1, int16_t x2, int16_t y2);

/*!
* @brief Do nothing for the given number of clock cycles.
*
* @param[volatile uint32_t wait] Number of clock cycles
*/
void TFT_LCD_delay (volatile uint32_t wait)
{
	while(wait--);
}

/*!
* @brief Transmit a byte through the pre-configured SPI interface.
*
* @param[uint8_t Val] 8-bit data
*/
void TFT_LCD_transmit_8bits (uint8_t val)
{
	/* Dummy variable to store the response */
	uint32_t dummy;

    /* Slave response is not needed*/
    LCD_CS_low();
    dummy = LPSPI0_write_8bits(val);
    LCD_CS_high();

    (void)dummy;
}

/*!
* @brief Write an 8-bit data while RS is high.
*
* @param[uint8_t data] 8-bit data
*/
void TFT_LCD_write_8data (uint8_t data)
{
	LCD_RS_data();											/* RS is high during data */
    TFT_LCD_transmit_8bits(data);							/* Transmit a byte through the pre-configured SPI interface */
}

/*!
* @brief Write a 16-bit data while RS is high.
*
* @param[uint16_t data] 16-bit data
*/
void TFT_LCD_write_16data (uint16_t data)
{
	LCD_RS_data();											/* RS is high during data */
    TFT_LCD_transmit_8bits((uint8_t)(data >> 8));			/* Transmit the first byte through the pre-configured SPI interface */
    TFT_LCD_transmit_8bits((uint8_t)data);					/* Transmit the second byte through the pre-configured SPI interface */
}

/*!
* @brief Write an 8-bit command while RS is low.
*
* @param[uint8_t cmd] 8-bit instruction
*/
void TFT_LCD_write_8command (uint8_t cmd)
{
	LCD_RS_command();										/* RS is low during instructions */
    TFT_LCD_transmit_8bits(cmd);							/* Transmit a byte through the pre-configured SPI interface */
}

/*!
* @brief Write a 16-bit command while RS is low.
*
* @param[uint16_t cmd] 16-bit instruction
*/
void TFT_LCD_write_16command (uint16_t cmd)
{
	LCD_RS_command();										/* RS is low during instructions */
    TFT_LCD_transmit_8bits((uint8_t)(cmd >> 8));			/* Transmit the first byte through the pre-configured SPI interface */
    TFT_LCD_transmit_8bits((uint8_t)cmd);					/* Transmit the second byte through the pre-configured SPI interface */
}

/*!
* @brief Write a 16-bit address followed by a 16-bit word.
*
* @param[uint16_t addr] 16-bit address/instruction
* @param[uint16_t data] 16-bit data
*/
void TFT_LCD_write_16register (uint16_t addr, uint16_t data)
{
	TFT_LCD_write_16command(addr); 							/* Write a 16-bit command while RS is low */
	TFT_LCD_write_16data(data);								/* Write a 16-bit data while RS is high */
}

/*!
* @brief Set an address window.
*
* @param[int16_t x1] 16-bit Start X coordinate
* @param[int16_t y1] 16-bit End X coordinate
* @param[int16_t x2] 16-bit Start Y coordinate
* @param[int16_t y2] 16-bit End Y coordinate
*/
void TFT_LCD_set_address_window (int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	TFT_LCD_write_8command(0x2A); 							/* ILI9341_CASET: Set Column Address */
	TFT_LCD_write_8data((uint8_t)(x1 >> 8));
	TFT_LCD_write_8data((uint8_t)x1);     					/* X-START */
	TFT_LCD_write_8data((uint8_t)(x2 >> 8));
	TFT_LCD_write_8data((uint8_t)x2);    					/* X-END */

	TFT_LCD_write_8command(0x2B); 							/* ILI9341_PASET: Set Row Address */
	TFT_LCD_write_8data((uint8_t)(y1 >> 8));
	TFT_LCD_write_8data((uint8_t)y1);     					/* Y-START */
	TFT_LCD_write_8data((uint8_t)(y2 >> 8));
	TFT_LCD_write_8data((uint8_t)y2);     					/* Y-END */

	TFT_LCD_write_8command(0x2C); 							/* ILI9341_RAMWR: Write to RAM */
}

/* Public Function Prototypes */

/*!
* @brief Initialize LCD Display. Controller: ILI9341.
*/
void LCD_display9341_init (void)
{
	LPSPI0_init_master();

	TFT_LCD_write_8command(0xEF);
	TFT_LCD_write_8data(0x03);
	TFT_LCD_write_8data(0x80);
	TFT_LCD_write_8data(0x02);

	TFT_LCD_write_8command(0xCF);
	TFT_LCD_write_8data(0x00);
	TFT_LCD_write_8data(0xC1);
	TFT_LCD_write_8data(0x30);

	TFT_LCD_write_8command(0xED);
	TFT_LCD_write_8data(0x64);
	TFT_LCD_write_8data(0x03);
	TFT_LCD_write_8data(0x12);
	TFT_LCD_write_8data(0x81);

	TFT_LCD_write_8command(0xE8);
	TFT_LCD_write_8data(0x85);
	TFT_LCD_write_8data(0x00);
	TFT_LCD_write_8data(0x78);

	TFT_LCD_write_8command(0xCB);
	TFT_LCD_write_8data(0x39);
	TFT_LCD_write_8data(0x2C);
	TFT_LCD_write_8data(0x00);
	TFT_LCD_write_8data(0x34);
	TFT_LCD_write_8data(0x02);

	TFT_LCD_write_8command(0xF7);
	TFT_LCD_write_8data(0x20);

	TFT_LCD_write_8command(0xEA);
	TFT_LCD_write_8data(0x00);
	TFT_LCD_write_8data(0x00);

	TFT_LCD_write_8command(0xC0); 							/* ILI9341_PWCTR1: Power Control */
	TFT_LCD_write_8data(0x23);    							/* VRH[5:0] */

	TFT_LCD_write_8command(0xC1); 							/* ILI9341_PWCTR2: Power Control */
	TFT_LCD_write_8data(0x10);    							/* SAP[2:0]; BT[3:0] */

	TFT_LCD_write_8command(0xC5); 							/* ILI9341_VMCTR1: VCM Control */
	TFT_LCD_write_8data(0x3e);
	TFT_LCD_write_8data(0x28);

	TFT_LCD_write_8command(0xC7); 							/* ILI9341_VMCTR2: VCM Control2 */
	TFT_LCD_write_8data(0x86);

	TFT_LCD_write_8command(0x36); 							/* ILI9341_MADCTL: Memory Access Control */
	TFT_LCD_write_8data(TFT_ORIGIN);

	TFT_LCD_write_8command(0x3A); 							/* ILI9341_PIXFMT */
	TFT_LCD_write_8data(0x55);

	TFT_LCD_write_8command(0xB1); 							/* ILI9341_FRMCTR1 */
	TFT_LCD_write_8data(0x00);
	TFT_LCD_write_8data(0x18);

	TFT_LCD_write_8command(0xB6); 							/* ILI9341_DFUNCTR: Display Function Control */
	TFT_LCD_write_8data(0x08);
	TFT_LCD_write_8data(0x82);
	TFT_LCD_write_8data(0x27);

	TFT_LCD_write_8command(0xF2); 							/* Gamma Function Disable */
	TFT_LCD_write_8data(0x00);

	TFT_LCD_write_8command(0x26); 							/* ILI9341_GAMMASET: Gamma Curve Selected */
	TFT_LCD_write_8data(0x01);

	TFT_LCD_write_8command(0xE0); 							/* ILI9341_GMCTRP1: Set Gamma */
	TFT_LCD_write_8data(0x0F);
	TFT_LCD_write_8data(0x31);
	TFT_LCD_write_8data(0x2B);
	TFT_LCD_write_8data(0x0C);
	TFT_LCD_write_8data(0x0E);
	TFT_LCD_write_8data(0x08);
	TFT_LCD_write_8data(0x4E);
	TFT_LCD_write_8data(0xF1);
	TFT_LCD_write_8data(0x37);
	TFT_LCD_write_8data(0x07);
	TFT_LCD_write_8data(0x10);
	TFT_LCD_write_8data(0x03);
	TFT_LCD_write_8data(0x0E);
	TFT_LCD_write_8data(0x09);
	TFT_LCD_write_8data(0x00);

	TFT_LCD_write_8command(0xE1); 							/* ILI9341_GMCTRN1  Set Gamma */
	TFT_LCD_write_8data(0x00);
	TFT_LCD_write_8data(0x0E);
	TFT_LCD_write_8data(0x14);
	TFT_LCD_write_8data(0x03);
	TFT_LCD_write_8data(0x11);
	TFT_LCD_write_8data(0x07);
	TFT_LCD_write_8data(0x31);
	TFT_LCD_write_8data(0xC1);
	TFT_LCD_write_8data(0x48);
	TFT_LCD_write_8data(0x08);
	TFT_LCD_write_8data(0x0F);
	TFT_LCD_write_8data(0x0C);
	TFT_LCD_write_8data(0x31);
	TFT_LCD_write_8data(0x36);
	TFT_LCD_write_8data(0x0F);

	TFT_LCD_write_8command(0x11); 							/* ILI9341_SLPOUT  Exit Sleep */
	TFT_LCD_delay(1000000);     							/* Wait for 60ms */
	TFT_LCD_write_8command(0x29); 							/* ILI9341_DISPON  Display On */

	LCD_fill_screen(BLACK);
}

/*!
* @brief Fill all the screen with the given color.
*
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_fill_screen (uint16_t color)
{
	TFT_LCD_set_address_window(0, 0, width-1, height-1);
	LCD_flood(color, (int32_t)TFT_WIDTH * (int32_t)TFT_HEIGHT);
}

/*!
* @brief Fill a section of the screen with the given color.
*
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
* @param[uint32_t length] Number of pixels, starts from the origin.
*/
void LCD_flood (uint16_t color, uint32_t length)
{
	uint16_t blocks;
	uint8_t i, color_high = color >> 8, color_low = color;

	/* Write first pixel normally, decrement counter by 1 */
	LCD_RS_data();
	TFT_LCD_transmit_8bits(color_high);
	TFT_LCD_transmit_8bits(color_low);
	length--;

	blocks = (uint16_t)(length / 64); 						/* 64 pixels/block */

	/* High and low bytes are identical. Leave prior data on the port(s) and just toggle the write strobe. */
	if (color_high == color_low)
	{
		while(blocks--)
		{
			i = 16; 										/* 64 pixels/block / 4 pixels/pass */
			do
			{
				TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low); TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low);
				TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low); TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low);
			}
			while(--i);
		}
		/* Fill any remaining pixels (1 to 64) */
		for (i = (uint8_t)length & 63; i--;)
		{
			TFT_LCD_transmit_8bits(color_high);
			TFT_LCD_transmit_8bits(color_low);
		}
	}
	else
	{
		while(blocks--)
		{
			i = 16; 										/* 64 pixels/block / 4 pixels/pass */
			do
			{
				TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low); TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low);
				TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low); TFT_LCD_transmit_8bits(color_high); TFT_LCD_transmit_8bits(color_low);
			}
			while(--i);
		}
		/* Fill any remaining pixels (1 to 64) */
		for (i = (uint8_t)length & 63; i--;)
		{
			TFT_LCD_transmit_8bits(color_high);
			TFT_LCD_transmit_8bits(color_low);
		}
	}
}

/*!
* @brief Fill only one pixel, with one of the available colors.
*
* @param[int16_t x] Pixel position. X Coordinate.
* @param[int16_t y] Pixel position. Y Coordinate.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_pixel (int16_t x, int16_t y, uint16_t color)
{
  /* Check if the pixel position is valid. Clip */
  if ((x < 0) || (y < 0) || (x >= width) || (y >= height)) return;

  TFT_LCD_set_address_window(x, y, x, y);
  LCD_flood(color, 1);
}

/*!
* @brief Draw a line from (x0, y0) to (x1, y0) with one of the available colors. Based on Bresenham's Algorithm - thx Wikpedia
*
* @param[int16_t x0] Initial X Coordinate.
* @param[int16_t y0] Initial Y Coordinate.
* @param[int16_t x1] Final X Coordinate.
* @param[int16_t y1] Final Y Coordinate.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_line (int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);

	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1)
	{
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t y_step;

	if (y0 < y1)
	{
		y_step = 1;
	}
	else
	{
		y_step = -1;
	}

	for (; x0 <= x1; x0++)
	{
		if (steep)
		{
			LCD_draw_pixel(y0, x0, color);
		}
		else
		{
			LCD_draw_pixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0)
		{
			y0 += y_step;
			err += dx;
		}
	}
}

/*!
* @brief Draw an horizontal line from (x, y) to (x+length, y) with one of the available colors.
*
* @param[int16_t x] Initial X Coordinate.
* @param[int16_t y] Initial Y Coordinate.
* @param[int16_t length] Line Length.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_fast_horizontal_line (int16_t x, int16_t y, int16_t length, uint16_t color)
{
	int16_t x2;

	/* Check if the desired position is valid. Off-screen clipping */
	if ((length <= 0) || (y < 0) || (y >= height) || (x >= width) || ((x2 = (x + length - 1)) < 0)) return;

	if (x < 0)        										/* Clip left */
	{
		length += x;
		x = 0;
	}

	if (x2 >= width)  										/* Clip right */
	{
		x2 = width - 1;
		length = x2 - x + 1;
	}

	TFT_LCD_set_address_window(x, y, x2, y);
	LCD_flood(color, length);
	TFT_LCD_set_address_window(0, 0, width - 1, height - 1);
}

/*!
* @brief Draw an horizontal line from (x, y) to (x, y+length) with one of the available colors.
*
* @param[int16_t x] Initial X Coordinate.
* @param[int16_t y] Initial Y Coordinate.
* @param[int16_t length] Line Length.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_fast_vertical_line (int16_t x, int16_t y, int16_t length, uint16_t color)
{
	int16_t y2;

	/* Check if the desired position is valid. Off-screen clipping */
	if ((length <= 0) || (x < 0) || (x >= width) || (y >= height) || ((y2 = (y + length - 1)) < 0)) return;

	if (y < 0)          									/* Clip top */
	{
		length += y;
		y = 0;
	}

	if (y2 >= height)  										/* Clip bottom */
	{
		y2 = height - 1;
		length = y2 - y + 1;
	}

	TFT_LCD_set_address_window(x, y, x, y2);
	LCD_flood(color, length);
	TFT_LCD_set_address_window(0, 0, width - 1, height - 1);
}

/*!
* @brief Print a single character.
*
* @param[int16_t x1] Initial X Coordinate.
* @param[int16_t y1] Initial Y Coordinate.
* @param[int8_t chr] Character to print.
* @param[uint16_t fg_color] Character color. Use the colors definitions in TFT_LCD.h
* @param[uint16_t bg_color] Background color (behind the character). Use the colors definitions in TFT_LCD.h
* @param[uint8_t size] Character font. Use the fonts definitions in TFT_LCD.h
*/
void LCD_draw_char (int16_t x1, int16_t y1, int8_t chr, uint16_t fg_color, uint16_t bg_color, uint8_t size)
{
	int16_t x2, y2;
	int8_t bit, array, dot, size_bit, size_array;

	x2 = x1 + (5 * (int16_t)size) - 1;
	y2 = y1 + (8 * (int16_t)size) - 1;
	TFT_LCD_set_address_window(x1, y1, x2, y2);

	for (bit = 0; bit < 8; bit++)
	{
		for (size_bit = 0; size_bit < size; size_bit++)
		{
			for (array = 0; array < 5; array++)
			{
				for (size_array = 0; size_array < size; size_array++)
				{
					dot = (font[(chr * 5) + array] >> (bit)) & 0x01;
					if (dot)
					{
						LCD_flood(fg_color, 1);
					}
					else
					{
						LCD_flood(bg_color, 1);
					}
				}
			}
		}
	}
}

/*!
* @brief Print a string.
*
* @param[int16_t x1] Initial X Coordinate.
* @param[int16_t y1] Initial Y Coordinate.
* @param[int8_t *chr] String to print.
* @param[uint16_t fg_color] String color. Use the colors definitions in TFT_LCD.h
* @param[uint16_t bg_color] Background color (behind the characters). Use the colors definitions in TFT_LCD.h
* @param[uint8_t size] String font. Use the fonts definitions in TFT_LCD.h
*/
void LCD_draw_string (int16_t x1, int16_t y1, int8_t *chr, uint16_t fg_color, uint16_t bg_color, uint8_t size)
{
	while (chr[0] != 0)
	{
		LCD_draw_char(x1, y1, chr[0], fg_color, bg_color, size);
		x1 += size * 6;
		chr++;
	}
}

/*!
* @brief Draw the given image file.
*
* @param[uint16_t pos_x] Initial X Coordinate, image origin.
* @param[uint16_t pos_y] Initial Y Coordinate, image origin.
* @param[uint16_t size_x] Image size in pixels.
* @param[uint16_t size_y] Image size in pixels
* @param[const uint8_t *image] Array of the image elements
*/
void LCD_draw_image (uint16_t pos_x, uint16_t pos_y, uint16_t size_x, uint16_t size_y, const uint8_t *image)
{
	uint32_t array_num = 0;

	TFT_LCD_set_address_window(pos_x, pos_y, (pos_x + size_x - 1), (pos_y + size_y - 1));

	for (pos_y = size_y; pos_y > 0; pos_y--)
	{
		for (pos_x = size_x; pos_x > 0; pos_x--)
		{
			TFT_LCD_write_8data(image[array_num++]);
			TFT_LCD_write_8data(image[array_num++]);
		}
	}
}

/*!
* @brief Draw a circle with the given ratio and given color.
*
* @param[int16_t x0] Initial X Coordinate, circle origin.
* @param[int16_t y0] Initial Y Coordinate, circle origin.
* @param[int16_t r] Ratio in pixels.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_circle (int16_t  x0, int16_t  y0, int16_t  r, uint16_t color)
{
	int16_t  f = 1 - r;
	int16_t  ddF_x = 1;
	int16_t  ddF_y = -2 * r;
	int16_t  x = 0;
	int16_t  y = r;

	LCD_draw_pixel(x0, y0 + r, color);
	LCD_draw_pixel(x0, y0 - r, color);
	LCD_draw_pixel(x0 + r, y0, color);
	LCD_draw_pixel(x0 - r, y0, color);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		LCD_draw_pixel(x0 + x, y0 + y, color);
		LCD_draw_pixel(x0 - x, y0 + y, color);
		LCD_draw_pixel(x0 + x, y0 - y, color);
		LCD_draw_pixel(x0 - x, y0 - y, color);
		LCD_draw_pixel(x0 + y, y0 + x, color);
		LCD_draw_pixel(x0 - y, y0 + x, color);
		LCD_draw_pixel(x0 + y, y0 - x, color);
		LCD_draw_pixel(x0 - y, y0 - x, color);
	}
}

/*!
* @brief Draw a circle "corner".
*
* @param[int16_t x0] Initial X Coordinate, circle origin.
* @param[int16_t y0] Initial Y Coordinate, circle origin.
* @param[int16_t r] Ratio in pixels.
* @param[uint8_t corner_name] Corner name.
* 		1 - Upper left
* 		2 - Upper right
* 		4 - Bottom right
* 		8 - Bottom left
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_circle_helper (int16_t x0, int16_t y0, int16_t r, uint8_t corner_name, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (corner_name & 0x4)								/* Bottom right */
		{
			LCD_draw_pixel(x0 + x, y0 + y, color);
			LCD_draw_pixel(x0 + y, y0 + x, color);
		}

		if (corner_name & 0x2)								/* Upper right */
		{
			LCD_draw_pixel(x0 + x, y0 - y, color);
			LCD_draw_pixel(x0 + y, y0 - x, color);
		}

		if (corner_name & 0x8)								/* Bottom left */
		{
			LCD_draw_pixel(x0 - y, y0 + x, color);
			LCD_draw_pixel(x0 - x, y0 + y, color);
		}

		if (corner_name & 0x1)								/* Upper left */
		{
			LCD_draw_pixel(x0 - y, y0 - x, color);
			LCD_draw_pixel(x0 - x, y0 - y, color);
		}
	}
}

/*!
* @brief Draw a simple square.
*
* @param[int16_t x] Initial X Coordinate, square origin.
* @param[int16_t y] Initial Y Coordinate, square origin.
* @param[int16_t l] Square size.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_square (int16_t x, int16_t y, int16_t l, uint16_t color)
{
	/* Smarter version */
	LCD_draw_fast_horizontal_line(x, y, l, color); 				/* Top */
	LCD_draw_fast_horizontal_line(x, y + l - 1, l, color); 		/* Bottom */
	LCD_draw_fast_vertical_line(x, y, l, color); 				/* Left */
	LCD_draw_fast_vertical_line(x + l - 1, y, l, color);      	/* Right */
}

/*!
* @brief Draw a simple rectangle.
*
* @param[int16_t x] Initial X Coordinate, rectangle origin.
* @param[int16_t y] Initial Y Coordinate, rectangle origin.
* @param[int16_t w] Rectangle width.
* @param[int16_t h] Rectangle height.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_rectangle (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	/* Smarter version */
	LCD_draw_fast_horizontal_line(x, y, w, color); 				/* Top */
	LCD_draw_fast_horizontal_line(x, y + h - 1, w, color); 		/* Bottom */
	LCD_draw_fast_vertical_line(x, y, h, color); 				/* Left */
	LCD_draw_fast_vertical_line(x + w - 1, y, h, color);     	/* Right */
}

/*!
* @brief Draw a rectangle with rounded corners.
*
* @param[int16_t x] Initial X Coordinate, rectangle origin.
* @param[int16_t y] Initial Y Coordinate, rectangle origin.
* @param[int16_t w] Rectangle width.
* @param[int16_t h] Rectangle height.
* @param[int16_t r] Ratio in pixels.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_round_rectangle (int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
	/* Smarter version */
	LCD_draw_fast_horizontal_line(x + r, y, w - 2 * r, color); 			/* Top */
	LCD_draw_fast_horizontal_line(x + r, y + h - 1, w - 2 * r, color); 	/* Bottom */
	LCD_draw_fast_vertical_line(x, y + r, h - 2 * r, color); 			/* Left */
	LCD_draw_fast_vertical_line(x + w - 1, y + r, h - 2 * r, color); 	/* Right */

	/* Draw four corners */
	LCD_draw_circle_helper(x + r, y + r, r, 1, color);
	LCD_draw_circle_helper(x + w - r - 1, y + r, r, 2, color);
	LCD_draw_circle_helper(x + w - r - 1, y + h - r - 1, r, 4, color);
	LCD_draw_circle_helper(x + r, y + h - r - 1, r, 8, color);
}

/*!
* @brief Draw a triangle.
*
* @param[int16_t x0] Triangle Vertex X Coordinate.
* @param[int16_t y0] Triangle Vertex Y Coordinate.
* @param[int16_t x1] Triangle Vertex X Coordinate.
* @param[int16_t y1] Triangle Vertex Y Coordinate.
* @param[int16_t x2] Triangle Vertex X Coordinate.
* @param[int16_t y2] Triangle Vertex Y Coordinate.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_draw_triangle (int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	LCD_draw_line(x0, y0, x1, y1, color);
	LCD_draw_line(x1, y1, x2, y2, color);
	LCD_draw_line(x2, y2, x0, y0, color);
}

/*!
* @brief Draw the given image file.
*
* @param[const uint8_t *image] In case the image size matches the LCD size, you can use this
* function with no more parameters that the image, instead of the draw_image function.
* Warning: If images with different sizes are printed they will be distorted.
*/
void LCD_fill_image (const uint8_t *image)
{
	LCD_draw_image(0, 0, width, height, image);
}

/*!
* @brief Draw and fill a circle with the given ratio and given color.
*
* @param[int16_t x0] Initial X Coordinate, circle origin.
* @param[int16_t y0] Initial Y Coordinate, circle origin.
* @param[int16_t r] Ratio in pixels.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_fill_circle (int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	LCD_draw_fast_vertical_line(x0, y0 - r, 2 * r + 1, color);
	LCD_fill_circle_helper(x0, y0, r, 3, 0, color);
}

/*!
* @brief Draw and fill a circle "corner".
*
* @param[int16_t x0] Initial X Coordinate, circle origin.
* @param[int16_t y0] Initial Y Coordinate, circle origin.
* @param[int16_t r] Ratio in pixels.
* @param[uint8_t corner_name] Corner name.
* 		1 - Right
* 		2 - Left
* @param[int16_t delta] Set circle proportions.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_fill_circle_helper (int16_t x0, int16_t y0, int16_t r, uint8_t corner_name, int16_t delta, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (corner_name & 0x1)								/* Right */
		{
			LCD_draw_fast_vertical_line(x0 + x, y0 - y, 2 * y + 1 + delta, color);
			LCD_draw_fast_vertical_line(x0 + y, y0 - x, 2 * x + 1 + delta, color);
		}

		if (corner_name & 0x2)								/* Left */
		{
			LCD_draw_fast_vertical_line(x0 - x, y0 - y, 2 * y + 1 + delta, color);
			LCD_draw_fast_vertical_line(x0 - y, y0 - x, 2 * x + 1 + delta, color);
		}
	}
}

/*!
* @brief Draw and fill a simple rectangle.
*
* @param[int16_t x1] Initial X Coordinate, rectangle origin.
* @param[int16_t y1] Initial Y Coordinate, rectangle origin.
* @param[int16_t w] Rectangle width.
* @param[int16_t h] Rectangle height.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_fill_rectangle (int16_t x1, int16_t y1, int16_t w, int16_t h, uint16_t color)
{
	int16_t x2, y2;

	/* Check if the desired position is valid. Off-screen clipping */
	if ((w <= 0) || (h <= 0) || (x1 >= width) || (y1 >= height) || ((x2 = x1 + w -1) < 0) || ((y2 = y1 + h -1) < 0 )) return;

	if (x1 < 0) 											/* Clip left */
	{
		w += x1;
		x1 = 0;
	}

	if (y1 < 0)  											/* Clip top */
	{
		h += y1;
		y1 = 0;
	}

	if (x2 >= width)  										/* Clip right  */
	{
		x2 = width - 1;
		w  = x2 - x1 + 1;
	}

	if (y2 >= height) 										/* Clip bottom */
	{
		y2 = height - 1;
		h  = y2 - y1 + 1;
	}

	TFT_LCD_set_address_window(x1, y1, x2, y2);
	LCD_flood(color, (uint32_t)w * (uint32_t)h);
	TFT_LCD_set_address_window(0, 0, width - 1, height - 1);
}

/*!
* @brief Draw a rectangle with rounded corners.
*
* @param[int16_t x] Initial X Coordinate, rectangle origin.
* @param[int16_t y] Initial Y Coordinate, rectangle origin.
* @param[int16_t w] Rectangle width.
* @param[int16_t h] Rectangle height.
* @param[int16_t r] Ratio in pixels.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_fill_round_rectangle (int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
	/* Smarter version */
	LCD_fill_rectangle(x + r, y, w - 2 * r, h, color);

	/* Draw four corners */
	LCD_fill_circle_helper(x + w - r -1, y + r, r, 1, h - 2 * r - 1, color);
	LCD_fill_circle_helper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}

/*!
* @brief Draw and fill a triangle.
*
* @param[int16_t x0] Triangle Vertex X Coordinate.
* @param[int16_t y0] Triangle Vertex Y Coordinate.
* @param[int16_t x1] Triangle Vertex X Coordinate.
* @param[int16_t y1] Triangle Vertex Y Coordinate.
* @param[int16_t x2] Triangle Vertex X Coordinate.
* @param[int16_t y2] Triangle Vertex Y Coordinate.
* @param[uint16_t color] Desired Color. Use the colors definitions in TFT_LCD.h
*/
void LCD_fill_triangle (int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	int16_t a, b, y, last;

	/* Sort coordinates by Y order (y2 >= y1 >= y0) */
	if (y0 > y1)
	{
		swap(y0, y1);
		swap(x0, x1);
	}

	if (y1 > y2)
	{
		swap(y2, y1);
		swap(x2, x1);
	}

	if (y0 > y1)
	{
		swap(y0, y1);
		swap(x0, x1);
	}

	/* Handle awkward all-on-same-line case as its own thing */
	if(y0 == y2)
	{
		a = b = x0;
		if(x1 < a)      a = x1;
		else if(x1 > b) b = x1;
		if(x2 < a)      a = x2;
		else if(x2 > b) b = x2;
		LCD_draw_fast_horizontal_line(a, y0, b-a+1, color);
		return;
	}

	int16_t
    	dx01 = x1 - x0,
		dy01 = y1 - y0,
		dx02 = x2 - x0,
		dy02 = y2 - y0,
		dx12 = x2 - x1,
		dy12 = y2 - y1,
		sa   = 0,
		sb   = 0;

	/* For upper part of triangle, find scan-line crossings for segments
	* 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scan-line y1
	* is included here (and second loop will be skipped, avoiding a /0
	* error there), otherwise scan-line y1 is skipped here and handled
	* in the second loop...which also avoids a /0 error here if y0=y1
	* (flat-topped triangle).
	*/
	if(y1 == y2) last = y1;   								/* Include y1 scan-line */
	else         last = y1 - 1; 							/* Skip it */

	for(y = y0; y <= last; y++)
	{
		a   = x0 + sa / dy01;
		b   = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		/* long-hand:
    	*  a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    	*  b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
		if(a > b) swap(a, b);
		LCD_draw_fast_horizontal_line(a, y, b - a + 1, color);
	}

	/* For lower part of triangle, find scan-line crossings for segments
	*  0-2 and 1-2.  This loop is skipped if y1=y2.
	*/
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);

	for(; y <= y2; y++) {
		a   = x1 + sa / dy12;
		b   = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		/* long-hand:
    	*  a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    	*  b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
		if(a > b) swap(a, b);
		LCD_draw_fast_horizontal_line(a, y, b - a + 1, color);
	}
}
