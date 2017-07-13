#ifndef __LCD_H
#define __LCD_H

#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdint.h"

void LCD_FillScreen(uint32_t color);
void LCD_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);
void LCD_DrawPixel(uint16_t x1, uint16_t y1, uint32_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);

void LCD_FillScreen_565(uint16_t color);
void LCD_FillRectangle_565(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawPixel_565(uint16_t x1, uint16_t y1, uint16_t color);
void LCD_DrawLine_565(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

void LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp);

#define swap(a,b) {int16_t t=a;a=b;b=t;}
#define convert24to32(x) (x|0xFF000000)

#endif
