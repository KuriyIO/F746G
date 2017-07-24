#ifndef __LTDC_H
#define __LTDC_H

#include "stm32f7xx_hal.h"
#include "string.h"
#include "stdint.h"
#include "fonts.h"

typedef struct
{
	uint32_t TextColor;
	uint32_t BackColor;
	sFONT *pFont;
}LTDC_DrawPropTypeDef;
//------------------------------------------
typedef enum
{
	CENTER_MODE = 0x01, /* Center mode */
	RIGHT_MODE = 0x02, /* Right mode */
	LEFT_MODE = 0x03, /* Left mode */
}Text_AlignModeTypedef;
//------------------------------------------

void LTDC_FillScreen(uint32_t color);
void LTDC_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);
void LTDC_DrawPixel(uint16_t x1, uint16_t y1, uint32_t color);
void LTDC_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);

void LTDC_FillScreen_565(uint16_t color);
void LTDC_FillRectangle_565(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LTDC_DrawPixel_565(uint16_t x1, uint16_t y1, uint16_t color);
void LTDC_DrawLine_565(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

void LTDC_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp);
void LTDC_DrawBitmapToMem(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp, uint8_t *pdst);

void DMA2D_LayersAlphaReconfig(uint32_t alpha1, uint32_t alpha2);

void LTDC_FonsIni(void);
void LTDC_SetFont(sFONT *fonts);
void LTDC_SetTextColor(uint32_t color);
void LTDC_SetBackColor(uint32_t color);
void LTDC_DrawChar(uint16_t x, uint16_t y, const uint8_t c);
void LTDC_DisplayString(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypedef Mode);

#define swap(a,b) {int16_t t=a;a=b;b=t;}
#define convert24to32(x) (x|0xFF000000)

//------------------------------------------
#define LCD_COLOR_BLUE ((uint32_t)0xFF0000FF)
#define LCD_COLOR_GREEN ((uint32_t)0xFF00FF00)
#define LCD_COLOR_RED ((uint32_t)0xFFFF0000)
#define LCD_COLOR_CYAN ((uint32_t)0xFF00FFFF)
#define LCD_COLOR_MAGENTA ((uint32_t)0xFFFF00FF)
#define LCD_COLOR_YELLOW ((uint32_t)0xFFFFFF00)
#define LCD_COLOR_LIGHTBLUE ((uint32_t)0xFF8080FF)
#define LCD_COLOR_LIGHTGREEN ((uint32_t)0xFF80FF80)
#define LCD_COLOR_LIGHTRED ((uint32_t)0xFFFF8080)
#define LCD_COLOR_LIGHTCYAN ((uint32_t)0xFF80FFFF)
#define LCD_COLOR_LIGHTMAGENTA ((uint32_t)0xFFFF80FF)
#define LCD_COLOR_LIGHTYELLOW ((uint32_t)0xFFFFFF80)
#define LCD_COLOR_DARKBLUE ((uint32_t)0xFF000080)
#define LCD_COLOR_DARKGREEN ((uint32_t)0xFF008000)
#define LCD_COLOR_DARKRED ((uint32_t)0xFF800000)
#define LCD_COLOR_DARKCYAN ((uint32_t)0xFF008080)
#define LCD_COLOR_DARKMAGENTA ((uint32_t)0xFF800080)
#define LCD_COLOR_DARKYELLOW ((uint32_t)0xFF808000)
#define LCD_COLOR_WHITE ((uint32_t)0xFFFFFFFF)
#define LCD_COLOR_LIGHTGRAY ((uint32_t)0xFFD3D3D3)
#define LCD_COLOR_GRAY ((uint32_t)0xFF808080)
#define LCD_COLOR_DARKGRAY ((uint32_t)0xFF404040)
#define LCD_COLOR_BLACK ((uint32_t)0xFF000000)
#define LCD_COLOR_BROWN ((uint32_t)0xFFA52A2A)
#define LCD_COLOR_ORANGE ((uint32_t)0xFFFFA500)
#define LCD_COLOR_TRANSPARENT ((uint32_t)0xFF000000)
//------------------------------------------

#endif
