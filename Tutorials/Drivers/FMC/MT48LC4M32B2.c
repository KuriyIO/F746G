#include "MT48LC4M32B2.h"

FMC_SDRAM_CommandTypeDef command;
HAL_StatusTypeDef hal_stat;


void MT48LC4M32B2_Init(SDRAM_HandleTypeDef *hsdram)
{
	__IO uint32_t tmpmrd = 0;	//temp Mode Register Data

	command.CommandMode 			= FMC_SDRAM_CMD_CLK_ENABLE;
	command.CommandTarget 			= FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber 		= 1;
	command.ModeRegisterDefinition 	= 0;
	hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);

	HAL_Delay(1);

	command.CommandMode 			= FMC_SDRAM_CMD_PALL;
	command.CommandTarget 			= FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber 		= 1;
	command.ModeRegisterDefinition 	= 0;
	hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);

	command.CommandMode 			= FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	command.CommandTarget 			= FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber 		= 8;
	command.ModeRegisterDefinition 	= 0;
	hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);

	tmpmrd = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1			|
					    SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   	|
					    SDRAM_MODEREG_CAS_LATENCY_2           	|
					    SDRAM_MODEREG_OPERATING_MODE_STANDARD 	|
					    SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	command.CommandMode 			= FMC_SDRAM_CMD_LOAD_MODE;
	command.CommandTarget 			= FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber 		= 1;
	command.ModeRegisterDefinition 	= tmpmrd;
	hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);

	/* Set the refresh rate counter */
	HAL_SDRAM_ProgramRefreshRate(hsdram, 0x0603);

}
