#include "ltdctest.h"
#include "fatfs.h"
#include "lcd.h"

#define LTDC_FRAME_BUFFER          ((uint32_t)0xC0000000)

extern RNG_HandleTypeDef hrng;
extern DMA2D_HandleTypeDef hdma2d;

extern uint8_t* bmp1;
extern uint8_t* dma2d_in1;
extern uint8_t* dma2d_in2;

extern uint8_t sect[4096];
extern uint32_t bytesread;
extern FIL MyFile; /* File object */

extern uint32_t OpenBMP(uint8_t *ptr, const char* fname);

void text_test(void)
{
	LTDC_FillScreen(0);

	LTDC_SetFont(&Font24);
	LTDC_SetTextColor(LCD_COLOR_CYAN);
	LTDC_SetBackColor(LCD_COLOR_BLACK);
	LTDC_DisplayString(14, 100, (uint8_t *)"STM32 Left 24", LEFT_MODE);

	LTDC_SetFont(&Font20);
	LTDC_SetTextColor(LCD_COLOR_RED);
	LTDC_DisplayString(14, 130, (uint8_t *)"STM32 Center 20", CENTER_MODE);

	LTDC_SetFont(&Font16);
	LTDC_SetTextColor(LCD_COLOR_MAGENTA);
	LTDC_DisplayString(1, 160, (uint8_t *)"STM32 Right 16", RIGHT_MODE);

	HAL_Delay(5000);
}

void bitmap_test(void)
{
	OpenBMP((uint8_t *)bmp1,"image01.bmp");
	LTDC_DrawBitmapToMem(0,0,(uint8_t *)bmp1,(uint8_t *)dma2d_in2);

	char str1[20] = {0};

	for(int j=1;j<=10;j++)
	{
		sprintf(str1,"image%02d.bmp",j);
		OpenBMP((uint8_t *)bmp1,str1);

		if(j%2!=0)
			LTDC_DrawBitmapToMem(0,0,(uint8_t *)bmp1,(uint8_t *)dma2d_in1);
		else
			LTDC_DrawBitmapToMem(0,0,(uint8_t *)bmp1,(uint8_t *)dma2d_in2);

		for(int i=0;i<=255;i++)
		{
			DMA2D_LayersAlphaReconfig(i,255-i);

			if(j%2!=0)
				HAL_DMA2D_BlendingStart_IT(&hdma2d, (uint32_t) dma2d_in1,
										   (uint32_t) dma2d_in2, LTDC_FRAME_BUFFER, 480, 272);
			else
				HAL_DMA2D_BlendingStart_IT(&hdma2d, (uint32_t) dma2d_in2,
										   (uint32_t) dma2d_in1, LTDC_FRAME_BUFFER, 480, 272);
			HAL_Delay(10);
		}
		HAL_Delay(3000);
	}
}

void ltdc_test(void)
{
	LTDC_FillScreen(0);

	for(int i=0;i<100;i++)
	{
		LTDC_FillScreen(HAL_RNG_GetRandomNumber(&hrng));
		HAL_Delay(100);
	}

	LTDC_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LTDC_FillRectangle(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LTDC_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<10000;i++)
	{
		LTDC_DrawPixel(HAL_RNG_GetRandomNumber(&hrng)%480,
					  HAL_RNG_GetRandomNumber(&hrng)%272,
					  HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(1);
	}

	LTDC_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<10000;i++)
	{
		for(int j=0;j<100;j++)
		{
			LTDC_DrawPixel(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,0);
		}

		LTDC_DrawPixel(HAL_RNG_GetRandomNumber(&hrng)%480,
					  HAL_RNG_GetRandomNumber(&hrng)%272,
					  HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(1);
	}

	LTDC_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LTDC_DrawLine(HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LTDC_FillScreen(0);
	HAL_Delay(1000);
}

void ltdc_test_565(void)
{
	LTDC_FillScreen_565(0);

	for(int i=0;i<100;i++)
	{
		LTDC_FillScreen_565((uint16_t)HAL_RNG_GetRandomNumber(&hrng));
		HAL_Delay(100);
	}

	LTDC_FillScreen_565(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LTDC_FillRectangle_565(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  (uint16_t)HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LTDC_FillScreen_565(0);
	HAL_Delay(1000);

	for(int i=0;i<10000;i++)
	{
		for(int j=0;j<100;j++)
		{
			LTDC_DrawPixel_565(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,0);
		}

		LTDC_DrawPixel_565(HAL_RNG_GetRandomNumber(&hrng)%480,
					  HAL_RNG_GetRandomNumber(&hrng)%272,
					  (uint16_t)HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(1);
	}

	LTDC_FillScreen_565(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LTDC_DrawLine_565(HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 (uint16_t)HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LTDC_FillScreen_565(0);
	HAL_Delay(1000);
}

uint32_t OpenBMP(uint8_t *ptr, const char* fname)
{
 uint32_t ind = 0, sz = 0, i1 = 0, ind1 = 0;
 static uint32_t bmp_addr;
 if(f_open(&MyFile, fname, FA_READ) != FR_OK)
 {
   LTDC_FillScreen(0xFFFF0000); //в случае неудачи окрасим экран в красный цвет
 }
 else
 {
   if (f_read (&MyFile, sect, 30, (UINT *)bytesread) != FR_OK)
   {
     Error_Handler();
   }
   else
   {
     bmp_addr = (uint32_t)sect;
     sz = *(uint16_t *) (bmp_addr + 2);
     sz |= (*(uint16_t *) (bmp_addr + 4)) << 16;

     /* Get bitmap data address offset */
     ind = *(uint16_t *) (bmp_addr + 10);
     ind |= (*(uint16_t *) (bmp_addr + 12)) << 16;
     f_close (&MyFile);
     f_open (&MyFile, fname, FA_READ);
     ind=0;
     do
     {
       if (sz < 4096)
       {
         i1 = sz;
       }
       else
       {
         i1 = 4096;
       }
       sz -= i1;
       f_lseek(&MyFile,ind1);
       f_read (&MyFile, sect, i1, (UINT *)&bytesread);

       memcpy((void*)(bmp1+ind1), (void*)sect, i1);
       ind1+=i1;
     }
     while (sz > 0);
     f_close (&MyFile);
   }
   ind1=0;
 }
 return 0;
}


