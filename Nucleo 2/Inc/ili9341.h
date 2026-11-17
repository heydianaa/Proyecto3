/*
 * ili9341.h
 *
 *  Created on: Aug 20, 2024
 *      Author: Pablo Mazariegos
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include "font.h"
#include <stdint.h>
#include "main.h"
#include "lcd_pins.h"

void LCD_Init(void);
uint8_t LCD_IsReady(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(char* text, int x, int y, int fontSize, int color, int background);

void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, const uint16_t *bitmap);
void LCD_BitmapTransparent(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *bitmap, uint16_t transparentColor);
void LCD_BitmapFast(unsigned int x, unsigned int y, unsigned int width, unsigned int height, const uint8_t *bitmap);
void LCD_Sprite(int x, int y, int width, int height, const uint16_t *bitmap,
                int columns, int index, char flip, char offset);

#endif /* INC_ILI9341_H_ */
