#include "lcd.h"
#include "stdlib.h"


extern LTDC_HandleTypeDef hltdc;
extern DMA2D_HandleTypeDef hdma2d;
uint16_t X_SIZE = 480;
uint16_t Y_SIZE = 272;

LCD_DrawPropTypeDef lcdprop;
const uint8_t *ch;

void LCD_FillScreen(uint32_t color)
{
	hdma2d.Init.Mode = DMA2D_R2M;
	hdma2d.Init.OutputOffset = 0;
	if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
	{
		if (HAL_DMA2D_Start(&hdma2d, color, hltdc.LayerCfg[0].FBStartAdress,
				 	 	 	hltdc.LayerCfg[0].ImageWidth, hltdc.LayerCfg[0].ImageHeight) == HAL_OK)
		{
			HAL_DMA2D_PollForTransfer(&hdma2d, 10);
		}
	}
}

void LCD_FillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color)
{
	if(x1>x2) swap(x1,x2);
	if(y1>y2) swap(y1,y2);
	uint32_t addr = 0;
	addr = (hltdc.LayerCfg[0].FBStartAdress) + 4*(y1*hltdc.LayerCfg[0].ImageWidth + x1);

	hdma2d.Init.Mode = DMA2D_R2M;
	hdma2d.Init.OutputOffset = hltdc.LayerCfg[0].ImageWidth-(x2-x1);

	if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
	{
		if (HAL_DMA2D_Start(&hdma2d, color, addr, x2-x1, y2-y1) == HAL_OK)
		{
			HAL_DMA2D_PollForTransfer(&hdma2d, 10);
		}
	}
}

void LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t color)
{
	*(__IO uint32_t*)(hltdc.LayerCfg[0].FBStartAdress+((sizeof(color))*(Ypos*hltdc.LayerCfg[0].ImageWidth+Xpos))) = color;
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color)
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

	if(y1<y2)
		ystep=1;
	else
		ystep=-1;

	for(;x1<=x2;x1++)
	{
		if(steep)
			LCD_DrawPixel(y1,x1,color);
		else
			LCD_DrawPixel(x1,y1,color);

		err-=dy;

		if(err<0)
		{
			y1 += ystep;
			err+=dx;
		}
	}
}

void LCD_FillScreen_565(uint16_t color)
{
	uint32_t i;
	uint32_t n = hltdc.LayerCfg[0].ImageHeight*hltdc.LayerCfg[0].ImageWidth;
	for(i=0;i<n;i++)
	{
		*(__IO uint16_t*)(hltdc.LayerCfg[0].FBStartAdress+(i*(sizeof(color)))) = color;
	}
}

void LCD_FillRectangle_565(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint32_t xpos,ypos;
	if(x1>x2) swap(x1,x2);
	if(y1>y2) swap(y1,y2);
	for(ypos=y1;ypos<=y2;ypos++)
	{
		for(xpos=x1;xpos<=x2;xpos++)
		{
			*(__IO uint16_t*)(hltdc.LayerCfg[0].FBStartAdress+((sizeof(color))*(ypos*hltdc.LayerCfg[0].ImageWidth+xpos)))= color;
		}
	}
}

void LCD_DrawPixel_565(uint16_t Xpos, uint16_t Ypos, uint16_t color)
{
	*(__IO uint16_t*)(hltdc.LayerCfg[0].FBStartAdress+(2*(Ypos*hltdc.LayerCfg[0].ImageWidth+Xpos)))= color;
}

void LCD_DrawLine_565(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
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
		if(steep) LCD_DrawPixel_565(y1,x1,color);
		else LCD_DrawPixel_565(x1,y1,color);
		err-=dy;
		if(err<0)
		{
			y1 += ystep;
			err+=dx;
		}
	}
}

void LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp)
{
	uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
	uint32_t address;

	/* Get bitmap data address offset */
	index = *(__IO uint16_t *) (pbmp + 10);
	index |= (*(__IO uint16_t *) (pbmp + 12)) << 16;

	/* Read bitmap width */
	width = *(uint16_t *) (pbmp + 18);
	width |= (*(uint16_t *) (pbmp + 20)) << 16;

	/* Read bitmap height */
	height = *(uint16_t *) (pbmp + 22);
	height |= (*(uint16_t *) (pbmp + 24)) << 16;

	/* Read bit/pixel */
	bit_pixel = *(uint16_t *) (pbmp + 28);

	/* Set the address */
	address = hltdc.LayerCfg[0].FBStartAdress + (((X_SIZE*Ypos) + Xpos)*4);

	/* Bypass the bitmap header */
	pbmp += (index + (width * (height-1) * (bit_pixel/8)));
	if((bit_pixel/8) == 4)
	{
		LCD_FillScreen(0xFFFF0000);
	}
	else if((bit_pixel/8) == 2)
	{
		LCD_FillScreen(0xFF00FF00);
	}
	else
	{
		/* Convert picture to ARGB8888 pixel format */
		for(index=0; index < height; index++)
		{
			hdma2d.Init.Mode = DMA2D_M2M_PFC;
			hdma2d.Init.ColorMode = DMA2D_ARGB8888;
			hdma2d.Init.OutputOffset = 0;
			hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
			hdma2d.LayerCfg[1].InputAlpha = 0xFF;
			hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB888;
			hdma2d.LayerCfg[1].InputOffset = 0;

			if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
			{
				if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
				{
					if (HAL_DMA2D_Start(&hdma2d, (uint32_t) pbmp, address, width, 1) == HAL_OK)
					{
						HAL_DMA2D_PollForTransfer(&hdma2d, 10);
					}
				}
			}

			address += X_SIZE*4;
			pbmp -= width*(bit_pixel/8);
		}
	}

	hdma2d.Init.Mode = DMA2D_M2M_BLEND;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	hdma2d.LayerCfg[0].InputOffset = 0;
	hdma2d.LayerCfg[0].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[0].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[0].InputAlpha = 0;

	if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
	{
		HAL_DMA2D_ConfigLayer(&hdma2d, 0);
	    HAL_DMA2D_ConfigLayer(&hdma2d, 1);
	}
}

void LCD_DrawBitmapToMem(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp, uint8_t *pdst)
{
	uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;

	/* Get bitmap data address offset */
	index = *(__IO uint16_t *) (pbmp + 10);
	index |= (*(__IO uint16_t *) (pbmp + 12)) << 16;

	/* Read bitmap width */
	width = *(uint16_t *) (pbmp + 18);
	width |= (*(uint16_t *) (pbmp + 20)) << 16;

	/* Read bitmap height */
	height = *(uint16_t *) (pbmp + 22);
	height |= (*(uint16_t *) (pbmp + 24)) << 16;

	/* Read bit/pixel */
	bit_pixel = *(uint16_t *) (pbmp + 28);

	/* Bypass the bitmap header */
	pbmp += (index + (width * (height-1) * (bit_pixel/8)));
	if((bit_pixel/8) == 4)
	{
		LCD_FillScreen(0xFFFF0000);
	}
	else if((bit_pixel/8) == 2)
	{
		LCD_FillScreen(0xFF00FF00);
	}
	else
	{
		/* Convert picture to ARGB8888 pixel format */
		for(index=0; index < height; index++)
		{
			hdma2d.Init.Mode = DMA2D_M2M_PFC;
			hdma2d.Init.ColorMode = DMA2D_ARGB8888;
			hdma2d.Init.OutputOffset = 0;
			hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
			hdma2d.LayerCfg[1].InputAlpha = 0xFF;
			hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB888;
			hdma2d.LayerCfg[1].InputOffset = 0;

			if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
			{
				if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
				{
					if (HAL_DMA2D_Start(&hdma2d, (uint32_t) pbmp, (uint32_t) pdst, width, 1) == HAL_OK)
					{
						HAL_DMA2D_PollForTransfer(&hdma2d, 10);
					}
				}
			}

			pdst += X_SIZE*4;
			pbmp -= width*(bit_pixel/8);
		}
	}

	hdma2d.Init.Mode = DMA2D_M2M_BLEND;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	hdma2d.LayerCfg[0].InputOffset = 0;
	hdma2d.LayerCfg[0].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[0].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[0].InputAlpha = 0;

	if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
	{
		HAL_DMA2D_ConfigLayer(&hdma2d, 0);
	    HAL_DMA2D_ConfigLayer(&hdma2d, 1);
	}
}

void DMA2D_LayersAlphaReconfig(uint32_t alpha1, uint32_t alpha2)
{
	hdma2d.LayerCfg[1].InputAlpha = alpha1;
	hdma2d.LayerCfg[0].InputAlpha = alpha2;
	HAL_DMA2D_ConfigLayer(&hdma2d, 1);
	HAL_DMA2D_ConfigLayer(&hdma2d, 0);
}

void FTF_FonsIni(void)
{
	lcdprop.BackColor=LCD_COLOR_BLACK;
	lcdprop.TextColor=LCD_COLOR_GREEN;
	lcdprop.pFont=&Font16;
}
//----------------------------------------
void LCD_SetFont(sFONT *fonts)
{
	lcdprop.pFont=fonts;
}
//----------------------------------------
void LCD_SetTextColor(uint32_t color)
{
	lcdprop.TextColor=color;
}
//----------------------------------------
void LCD_SetBackColor(uint32_t color)
{
	lcdprop.BackColor=color;
}
//----------------------------------------
void LCD_DrawChar(uint16_t x, uint16_t y, const uint8_t c)
{

	uint32_t i=0, j=0;
	uint16_t height, width;
	uint8_t offset;
	uint8_t *pchar;
	uint32_t line;
	ch = &lcdprop.pFont->table[(c-' ') * lcdprop.pFont->Height *
		((lcdprop.pFont->Width +7) / 8)];
	height = lcdprop.pFont->Height;
	width = lcdprop.pFont->Width;
	offset = 8 * ((width + 7)/8) - width;
	for(i = 0;i < height; i++)
	{
		pchar = ((uint8_t *)ch + (width + 7)/8 * i);
		switch((width + 7)/8)
		{
		case 1:
			line = pchar[0];
			break;
		case 2:
			line = (pchar[0]<<8) | pchar[1];
			break;
		case 3:
		default:
			line = (pchar[0]<<16) | (pchar[1]<<8) | pchar[2];
			break;
		}
		for(j = 0;j < width; j++)
		{
			if(line & (1 << (width - j + offset -1)))
			{
				LCD_DrawPixel((x+j), y, lcdprop.TextColor);
			}
			else
			{
				LCD_DrawPixel((x+j), y, lcdprop.BackColor);
			}
		}
		y++;
	}
}
//----------------------------------------
void LCD_DisplayString(uint16_t Xpos, uint16_t Ypos, const uint8_t *Text,
		Text_AlignModeTypedef Mode)

{
	uint16_t ref_column = 1, i = 0;
	uint32_t size = 0, xsize = 0;
	uint8_t *ptr = Text;
	while(*ptr++) size++ ;
	xsize = (X_SIZE/lcdprop.pFont->Width);
	switch (Mode)
	{
		case CENTER_MODE:
		{
			ref_column = Xpos + ((xsize-size) * lcdprop.pFont->Width) / 2;
			break;
		}
		case LEFT_MODE:
		{
			ref_column = Xpos;
			break;
		}
		case RIGHT_MODE:
		{
			ref_column = Xpos + ((xsize-size) * lcdprop.pFont->Width);
			break;
		}
		default:
		{
			ref_column = Xpos;
			break;
		}
	}
	if((ref_column < 1) || (ref_column >= 0x8000))
	{
		ref_column = 1;
	}
	while ((*Text != 0) & (((X_SIZE - (i*lcdprop.pFont->Width)) & 0xFFFF) >=
			lcdprop.pFont->Width))
	{
		LCD_DrawChar(ref_column, Ypos, *Text);
		ref_column += lcdprop.pFont->Width;
		Text++;
		i++;
	}
}
