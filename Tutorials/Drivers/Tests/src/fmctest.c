#include "fmctest.h"

static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset)
{
	uint32_t tmpIndex = 0;

	/* Put in global buffer different values */
	for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
	{
		pBuffer[tmpIndex] = tmpIndex + uwOffset;
	}
}

void fmc_test(UART_HandleTypeDef *huart)
{
	uint32_t aTxBuffer[BUFFER_SIZE];
	uint32_t aRxBuffer[BUFFER_SIZE];
	uint32_t writeBackBuffer[BUFFER_SIZE];
	uint32_t uwIndex;

	/* Starting test */
	char testtmpstr[20] = {0};
	sprintf(testtmpstr,"FMC TEST STARTED\r\n");
	HAL_UART_Transmit(huart, (uint8_t*)testtmpstr,strlen(testtmpstr),0x1000);
	HAL_Delay(100);

	/* Saving data for backup */
	for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
	{
		writeBackBuffer[uwIndex] = *(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*uwIndex);
	}

	/* init buffer with some values */
	Fill_Buffer(aTxBuffer, BUFFER_SIZE, 0xDEADC0DE);

	/* check single write/read */
	*(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR) = 0x12345678;
	uint32_t tmp = *(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR);
	if(tmp == 0)
	{
		sprintf(testtmpstr,"FMC TEST FAILED! SINGLE WRITE/READ ERROR!\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)testtmpstr,strlen(testtmpstr),0x1000);
		HAL_Delay(100);
		return;
	}

	/* load SDRAM from TX buffer*/
	for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
	{
		*(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*uwIndex) = aTxBuffer[uwIndex];
	}

	/* read SDRAM to RX buffer */
	for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
	{
		aRxBuffer[uwIndex] = *(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*uwIndex);
	}

	/* readed data must be the same as writed */
	for(uint16_t i = 0; i < BUFFER_SIZE; i++)
	{
		if(aTxBuffer[i] != aRxBuffer[i])
		{
			sprintf(testtmpstr,"FMC TEST FAILED! BUFFER WRITE/READ ERROR!\r\n");
			HAL_UART_Transmit(huart, (uint8_t*)testtmpstr,strlen(testtmpstr),0x1000);
			HAL_Delay(100);
			return;
		}
	}

	/* uncomment for print values to debug uart */
//	for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
//	{
//		sprintf(testtmpstr,"%03ld: 0x%08lX\r\n",(unsigned long) uwIndex,(unsigned long)aRxBuffer[uwIndex]);
//		HAL_UART_Transmit(huart, (uint8_t*)testtmpstr,strlen(testtmpstr),0x1000);
//		HAL_Delay(100);
//	}

	/* Restoring data from backup */
	for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
	{
		*(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*uwIndex) = writeBackBuffer[uwIndex];
	}

	sprintf(testtmpstr,"FMC TEST PASSED OK!\r\n");
	HAL_UART_Transmit(huart, (uint8_t*)testtmpstr,strlen(testtmpstr),0x1000);
	HAL_Delay(100);
}
