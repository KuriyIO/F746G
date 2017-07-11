#include "ltdctest.h"

void ltdc_test(LTDC_HandleTypeDef *hltdc, volatile uint32_t *LTDC_MEM, RNG_HandleTypeDef *hrng)
{
	HAL_LTDC_SetAddress(hltdc, LTDC_MEM, 0);
	LCD_FillScreen(0);

	for(int i=0;i<100;i++)
	{
		LCD_FillScreen((uint16_t)HAL_RNG_GetRandomNumber(hrng));
		HAL_Delay(100);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LCD_FillRectangle(HAL_RNG_GetRandomNumber(hrng)%480,
						  HAL_RNG_GetRandomNumber(hrng)%272,
						  HAL_RNG_GetRandomNumber(hrng)%480,
						  HAL_RNG_GetRandomNumber(hrng)%272,
						  (uint16_t)HAL_RNG_GetRandomNumber(hrng));

		HAL_Delay(10);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<10000;i++)
	{
		for(int j=0;j<100;j++)
		{
			LCD_DrawPixel(HAL_RNG_GetRandomNumber(hrng)%480,
						  HAL_RNG_GetRandomNumber(hrng)%272,0);
		}

		LCD_DrawPixel(HAL_RNG_GetRandomNumber(hrng)%480,
					  HAL_RNG_GetRandomNumber(hrng)%272,
					  (uint16_t)HAL_RNG_GetRandomNumber(hrng));

		HAL_Delay(1);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);

	for(int i=0;i<1000;i++)
	{
		LCD_DrawLine(HAL_RNG_GetRandomNumber(hrng)%480,
					 HAL_RNG_GetRandomNumber(hrng)%272,
					 HAL_RNG_GetRandomNumber(hrng)%480,
					 HAL_RNG_GetRandomNumber(hrng)%272,
					 (uint16_t)HAL_RNG_GetRandomNumber(hrng));

		HAL_Delay(10);
	}

	LCD_FillScreen(0);
	HAL_Delay(1000);
}
