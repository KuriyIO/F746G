#include "ltdctest.h"

extern RNG_HandleTypeDef hrng;

void ltdc_test(void)
{
	LCD_FillScreen(0);

	for(int i=0;i<100;i++)
	{
		LCD_FillScreen(HAL_RNG_GetRandomNumber(&hrng));
		HAL_Delay(100);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LCD_FillRectangle(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<10000;i++)
	{
		LCD_DrawPixel(HAL_RNG_GetRandomNumber(&hrng)%480,
					  HAL_RNG_GetRandomNumber(&hrng)%272,
					  HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(1);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<10000;i++)
	{
		for(int j=0;j<100;j++)
		{
			LCD_DrawPixel(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,0);
		}

		LCD_DrawPixel(HAL_RNG_GetRandomNumber(&hrng)%480,
					  HAL_RNG_GetRandomNumber(&hrng)%272,
					  HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(1);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LCD_DrawLine(HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);
}

void ltdc_test_565(void)
{
	LCD_FillScreen_565(0);

	for(int i=0;i<100;i++)
	{
		LCD_FillScreen_565((uint16_t)HAL_RNG_GetRandomNumber(&hrng));
		HAL_Delay(100);
	}

	LCD_FillScreen_565(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LCD_FillRectangle_565(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,
						  (uint16_t)HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LCD_FillScreen_565(0);
	HAL_Delay(1000);

	for(int i=0;i<10000;i++)
	{
		for(int j=0;j<100;j++)
		{
			LCD_DrawPixel_565(HAL_RNG_GetRandomNumber(&hrng)%480,
						  HAL_RNG_GetRandomNumber(&hrng)%272,0);
		}

		LCD_DrawPixel_565(HAL_RNG_GetRandomNumber(&hrng)%480,
					  HAL_RNG_GetRandomNumber(&hrng)%272,
					  (uint16_t)HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(1);
	}

	LCD_FillScreen_565(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LCD_DrawLine_565(HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 HAL_RNG_GetRandomNumber(&hrng)%480,
					 HAL_RNG_GetRandomNumber(&hrng)%272,
					 (uint16_t)HAL_RNG_GetRandomNumber(&hrng));

		HAL_Delay(10);
	}

	LCD_FillScreen_565(0);
	HAL_Delay(1000);
}
