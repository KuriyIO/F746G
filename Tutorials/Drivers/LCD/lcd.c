#include "lcd.h"


extern LTDC_HandleTypeDef hltdc;

void LCD_FillScreen(uint32_t color)
{
	uint32_t i;
	uint32_t n = hltdc.LayerCfg[0].ImageHeight*hltdc.LayerCfg[0].ImageWidth;
	for(i=0;i<n;i++)
	{
		*(__IO uint16_t*)(hltdc.LayerCfg[0].FBStartAdress+(i*2))=(uint16_t) color;
	}
}

void LCD_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint32_t xpos,ypos;
	if(x1>x2) swap(x1,x2);
	if(y1>y2) swap(y1,y2);
	for(ypos=y1;ypos<=y2;ypos++)
	{
		for(xpos=x1;xpos<=x2;xpos++)
		{
			*(__IO uint16_t*)(hltdc.LayerCfg[0].FBStartAdress+(2*(ypos*hltdc.LayerCfg[0].ImageWidth+xpos)))=(uint16_t) color;
		}
	}
}

void LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t color)
{
	*(__IO uint16_t*)(hltdc.LayerCfg[0].FBStartAdress+(2*(Ypos*hltdc.LayerCfg[0].ImageWidth+Xpos)))=(uint16_t) color;
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	int steep = abs(y2-y1)>abs(x2-x1);
	if(steep)
	{
		swap(x1,y1);
		swap(x2,y2);
	}
	if(x1>x2)
	{
		swap(x1,x2);
		swap(y1,y2);
	}
	int dx,dy;
	dx=x2-x1;
	dy=abs(y2-y1);
	int err=dx/2;
	int ystep;
	if(y1<y2) ystep=1;
	else ystep=-1;
	for(;x1<=x2;x1++)
	{
		if(steep) LCD_DrawPixel(y1,x1,color);
		else LCD_DrawPixel(x1,y1,color);
		err-=dy;
		if(err<0)
		{
			y1 += ystep;
			err+=dx;
		}
	}
}
