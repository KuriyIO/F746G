#include "tstest.h"
#include "lcd.h"
#include "ft5336.h"

extern TS_StateTypeDef TS_State;
uint16_t x=0, y=0;
static uint32_t tscnt[5]={0};
static uint16_t xstart[5]={0}, ystart[5]={0};

void ts_test(void)
{
	LTDC_FillScreen(LCD_COLOR_BLACK);
	LTDC_SetFont(&Font20);
	LTDC_SetTextColor(LCD_COLOR_MAGENTA);
	LTDC_SetBackColor(LCD_COLOR_BLACK);
	LTDC_DisplayString(0, 5, (uint8_t *)"TS TEST", CENTER_MODE);
	LTDC_SetFont(&Font16);
	LTDC_SetTextColor(LCD_COLOR_DARKGREEN);
	LTDC_DisplayString(0, 27, (uint8_t *)"Click the custom button to continue", CENTER_MODE);
	while(CheckForUserInput()==0)
	{
		TS_GetState(&TS_State);
		if(TS_State.touchDetected)
		{
			x = TS_State.touchX[0];
			y = TS_State.touchY[0];

			LTDC_SetFont(&Font20);
			LTDC_SetTextColor(LCD_COLOR_CYAN);
			LTDC_SetBackColor(LCD_COLOR_BLACK);

			if(tscnt[0]>0)
			{
				LTDC_DrawLine(xstart[0],ystart[0],x,y,LCD_COLOR_RED);
			}
			else
			{
				LTDC_DrawPixel(x,y,LCD_COLOR_RED);
			}

			xstart[0]=x; ystart[0]=y;
			tscnt[0]++;
			char tmpstr[30] = {0};
			sprintf(tmpstr,"1: x=%03d; y=%03d",x,y);
			LTDC_DisplayString(14, 50, (uint8_t *)tmpstr, LEFT_MODE);

			if (TS_State.touchDetected >= 2)
			{
				x = TS_State.touchX[1];
				y = TS_State.touchY[1];

				if(tscnt[1]>0)
				{
					LTDC_DrawLine(xstart[1],ystart[1],x,y,LCD_COLOR_GREEN);
				}
				else
				{
					LTDC_DrawPixel(x,y,LCD_COLOR_GREEN);
				}

				xstart[1]=x; ystart[1]=y;
				tscnt[1]++;
				sprintf(tmpstr, "2: x=%03d; y=%03d", x, y);
				LTDC_DisplayString(14, 80, (uint8_t *)tmpstr, LEFT_MODE);
			}
			else
			{
				LTDC_FillRectangle(14,80,250,109,LCD_COLOR_BLACK);
			}

			if (TS_State.touchDetected >= 3)
			{
				x = TS_State.touchX[2];
				y = TS_State.touchY[2];
				if(tscnt[2]>0)
				{
					LTDC_DrawLine(xstart[2],ystart[2],x,y,LCD_COLOR_YELLOW);
				}
				else
				{
					LTDC_DrawPixel(x,y,LCD_COLOR_YELLOW);
				}

				xstart[2]=x; ystart[2]=y;
				tscnt[2]++;
				sprintf(tmpstr, "3: x=%03d; y=%03d", x, y);
				LTDC_DisplayString(14, 110, (uint8_t *)tmpstr, LEFT_MODE);
			}
			else
			{
				LTDC_FillRectangle(14,110,250,139,LCD_COLOR_BLACK);
			}

			if (TS_State.touchDetected >= 4)
			{
				x = TS_State.touchX[3];
				y = TS_State.touchY[3];
				if(tscnt[3]>0)
				{
					LTDC_DrawLine(xstart[3],ystart[3],x,y,LCD_COLOR_ORANGE);
				}
				else
				{
					LTDC_DrawPixel(x,y,LCD_COLOR_ORANGE);
				}
				xstart[3]=x; ystart[3]=y;
				tscnt[3]++;
				sprintf(tmpstr, "4: x=%03d; y=%03d", x, y);
				LTDC_DisplayString(14, 140, (uint8_t *)tmpstr, LEFT_MODE);
			}
			else
			{
				LTDC_FillRectangle(14,140,250,169,LCD_COLOR_BLACK);
			}

			if (TS_State.touchDetected >= 5)
			{
				x = TS_State.touchX[4];
				y = TS_State.touchY[4];
				if(tscnt[4]>0)
				{
					LTDC_DrawLine(xstart[4],ystart[4],x,y,LCD_COLOR_BLUE);
				}
				else
				{
					LTDC_DrawPixel(x,y,LCD_COLOR_BLUE);
				}

				xstart[4]=x; ystart[4]=y;
				tscnt[4]++;
				sprintf(tmpstr, "5: x=%03d; y=%03d", x, y);
				LTDC_DisplayString(14, 170, (uint8_t *)tmpstr, LEFT_MODE);
			}
			else
			{
				LTDC_FillRectangle(14,170,250,199,LCD_COLOR_BLACK);
			}
		}
		else
		{
			tscnt[0]=0; tscnt[1]=0; tscnt[2]=0; tscnt[3]=0; tscnt[4]=0;
		}
		HAL_Delay(10);
	}
	LTDC_FillScreen(LCD_COLOR_BLACK);
}

uint8_t CheckForUserInput(void)
{
  if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11) != GPIO_PIN_RESET)
  {
    HAL_Delay(10);
    while (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11) != GPIO_PIN_RESET);
    return 1 ;
  }
  return 0;
}
