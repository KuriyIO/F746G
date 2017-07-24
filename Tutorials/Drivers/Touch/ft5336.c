#include "ft5336.h"
#include "lcd.h"

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;

static ft5336_handle_TypeDef ft5336_handle = {FT5336_I2C_NOT_INITIALIZED, 0, 0};
static uint8_t tsOrientation;

void TouchError(char* msg)
{
	LCD_FillScreen(LCD_COLOR_RED);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg,strlen(msg),0x1000);
	HAL_Delay(1000);
}

void TouchInit(void)
{
	uint8_t regValue = 0;
	uint8_t readId = TouchReadID(TS_I2C_ADDRESS);
	if(readId==0)
	{
		char tmpstr[20] = {0};
		sprintf(tmpstr,"Fail ReadID: ID read as %d\r\n", readId);
		TouchError(tmpstr);
	}
	tsOrientation = TS_SWAP_XY;
	//Configure the FT5336 device to stop generating IT on the given INT pin
	regValue = (FT5336_G_MODE_INTERRUPT_POLLING & (FT5336_G_MODE_INTERRUPT_MASK >> FT5336_G_MODE_INTERRUPT_SHIFT)) << FT5336_G_MODE_INTERRUPT_SHIFT;
	TouchIOWrite(TS_I2C_ADDRESS, FT5336_GMODE_REG, regValue);

	ft5336_handle.i2cInitialized = FT5336_I2C_INITIALIZED;

	HAL_Delay(200);
}

uint8_t TouchIORead(uint8_t Addr, uint8_t Reg)
{
	uint8_t read_value = 0;
	HAL_I2C_Mem_Read(&hi2c3, (uint16_t) Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_8BIT, &read_value, 1, 0x400);
	return read_value;
}

uint8_t TouchReadID(uint8_t DevAddr)
{
	uint8_t ReadID = 0;
	uint8_t Attempts = 0, IsFound = 0;
	for(Attempts = 0; Attempts <3 && IsFound == 0; Attempts++)
	{
		ReadID = TouchIORead(DevAddr, FT5336_CHIP_ID_REG);
		if(ReadID == FT5336_ID_VALUE)
			IsFound = 1;
	}
	return ReadID;
}

void TouchIOWrite(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	if(HAL_I2C_Mem_Write(&hi2c3, (uint16_t) Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x400) != HAL_OK)
	{
		TouchError("Error! HAL is not HAL_OK while sending byte to touch");
	}
}

uint8_t TS_GetState(TS_StateTypeDef *TS_State)
{
	static uint32_t _x[TS_MAX_NB_TOUCH] = {0, 0};
	static uint32_t _y[TS_MAX_NB_TOUCH] = {0, 0};
	uint8_t ts_status = TS_OK;
	uint16_t x[TS_MAX_NB_TOUCH];
	uint16_t y[TS_MAX_NB_TOUCH];
	uint16_t brute_x[TS_MAX_NB_TOUCH];
	uint16_t brute_y[TS_MAX_NB_TOUCH];
	uint16_t x_diff;
	uint16_t y_diff;
	uint32_t index;
	uint32_t weight = 0;
	uint32_t area = 0;
	uint32_t event = 0;

	TS_State->touchDetected = TS_DetectTouch(TS_I2C_ADDRESS);
	if(TS_State->touchDetected)
	{
		for(index=0; index < TS_State->touchDetected; index++)
		{
			TS_GetXY(TS_I2C_ADDRESS, &(brute_x[index]), &(brute_y[index]));
			if(tsOrientation == TS_SWAP_NONE)
			{
			  x[index] = brute_x[index];
			  y[index] = brute_y[index];
			}
			if(tsOrientation & TS_SWAP_X)
			{
			  x[index] = 4096 - brute_x[index];
			}
			if(tsOrientation & TS_SWAP_Y)
			{
			  y[index] = 4096 - brute_y[index];
			}
			if(tsOrientation & TS_SWAP_XY)
			{
			  y[index] = brute_x[index];
			  x[index] = brute_y[index];
			}
			x_diff = x[index] > _x[index]? (x[index] - _x[index]): (_x[index] - x[index]);
			y_diff = y[index] > _y[index]? (y[index] - _y[index]): (_y[index] - y[index]);
			if ((x_diff + y_diff) > 5)
			{
				_x[index] = x[index];
				_y[index] = y[index];
			}
			TS_State->touchX[index] = x[index];
			TS_State->touchY[index] = y[index];

			TS_GetTouchInfo(TS_I2C_ADDRESS, index, &weight, &area, &event);
			TS_State->touchWeight[index] = weight;
			TS_State->touchArea[index] = area;

			switch(event)
			{
				case FT5336_TOUCH_EVT_FLAG_PRESS_DOWN :
					TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
					break;
				case FT5336_TOUCH_EVT_FLAG_LIFT_UP :
					TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
					break;
				case FT5336_TOUCH_EVT_FLAG_CONTACT :
					TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
					break;
				case FT5336_TOUCH_EVT_FLAG_NO_EVENT :
					TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
					break;
				default :
					ts_status = TS_ERROR;
					break;
			}
		}

		ts_status = TS_Get_GestureId(TS_State);
	}
	return (ts_status);
}

uint8_t TS_DetectTouch(uint16_t DeviceAddr)
{
	uint8_t nbTouch = 0;
	nbTouch = TouchIORead(DeviceAddr, FT5336_TD_STAT_REG);
	nbTouch &= FT5336_TD_STAT_MASK;

	if(nbTouch > FT5336_MAX_DETECTABLE_TOUCH)
	{
		nbTouch = 0;
	}

	ft5336_handle.currActiveTouchNb = nbTouch;
	ft5336_handle.currActiveTouchIdx = 0;

	return (nbTouch);
}

void TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
 volatile uint8_t ucReadData = 0;
 static uint16_t coord;
 uint8_t regAddressXLow = 0;
 uint8_t regAddressXHigh = 0;
 uint8_t regAddressYLow = 0;
 uint8_t regAddressYHigh = 0;
 if(ft5336_handle.currActiveTouchIdx < ft5336_handle.currActiveTouchNb)
 {
	switch(ft5336_handle.currActiveTouchIdx)
	{
	  case 0 :
	    regAddressXLow = FT5336_P1_XL_REG;
	    regAddressXHigh = FT5336_P1_XH_REG;
	    regAddressYLow = FT5336_P1_YL_REG;
	    regAddressYHigh = FT5336_P1_YH_REG;
	    break;
	  case 1 :
	    regAddressXLow = FT5336_P2_XL_REG;
	    regAddressXHigh = FT5336_P2_XH_REG;
	    regAddressYLow = FT5336_P2_YL_REG;
	    regAddressYHigh = FT5336_P2_YH_REG;
	    break;
	  case 2 :
		regAddressXLow = FT5336_P3_XL_REG;
	    regAddressXHigh = FT5336_P3_XH_REG;
	    regAddressYLow = FT5336_P3_YL_REG;
	    regAddressYHigh = FT5336_P3_YH_REG;
	    break;
	  case 3 :
	    regAddressXLow = FT5336_P4_XL_REG;
	    regAddressXHigh = FT5336_P4_XH_REG;
	    regAddressYLow = FT5336_P4_YL_REG;
	    regAddressYHigh = FT5336_P4_YH_REG;
	    break;
	  case 4 :
	    regAddressXLow = FT5336_P5_XL_REG;
	    regAddressXHigh = FT5336_P5_XH_REG;
	    regAddressYLow = FT5336_P5_YL_REG;
	    regAddressYHigh = FT5336_P5_YH_REG;
	    break;
	  case 5 :
	    regAddressXLow = FT5336_P6_XL_REG;
	    regAddressXHigh = FT5336_P6_XH_REG;
	    regAddressYLow = FT5336_P6_YL_REG;
	    regAddressYHigh = FT5336_P6_YH_REG;
	    break;
	  case 6 :
	    regAddressXLow = FT5336_P7_XL_REG;
	    regAddressXHigh = FT5336_P7_XH_REG;
	    regAddressYLow = FT5336_P7_YL_REG;
	    regAddressYHigh = FT5336_P7_YH_REG;
	    break;
	  case 7 :
	    regAddressXLow = FT5336_P8_XL_REG;
	    regAddressXHigh = FT5336_P8_XH_REG;
	    regAddressYLow = FT5336_P8_YL_REG;
	    regAddressYHigh = FT5336_P8_YH_REG;
	    break;
	  case 8 :
	    regAddressXLow = FT5336_P9_XL_REG;
	    regAddressXHigh = FT5336_P9_XH_REG;
	    regAddressYLow = FT5336_P9_YL_REG;
	    regAddressYHigh = FT5336_P9_YH_REG;
	    break;
	  case 9 :
	    regAddressXLow = FT5336_P10_XL_REG;
	    regAddressXHigh = FT5336_P10_XH_REG;
	    regAddressYLow = FT5336_P10_YL_REG;
	    regAddressYHigh = FT5336_P10_YH_REG;
	    break;
	  default :
	    break;
	}

	ucReadData = TouchIORead(DeviceAddr, regAddressXLow);
	coord = (ucReadData & FT5336_TOUCH_POS_LSB_MASK) >> FT5336_TOUCH_POS_LSB_SHIFT;
	ucReadData = TouchIORead(DeviceAddr, regAddressXHigh);
	coord |= ((ucReadData & FT5336_TOUCH_POS_MSB_MASK) >> FT5336_TOUCH_POS_MSB_SHIFT) << 8;
	*X = coord;
	ucReadData = TouchIORead(DeviceAddr, regAddressYLow);
	coord = (ucReadData & FT5336_TOUCH_POS_LSB_MASK) >> FT5336_TOUCH_POS_LSB_SHIFT;
	ucReadData = TouchIORead(DeviceAddr, regAddressYHigh);
	coord |= ((ucReadData & FT5336_TOUCH_POS_MSB_MASK) >> FT5336_TOUCH_POS_MSB_SHIFT) << 8;
	*Y = coord;

	ft5336_handle.currActiveTouchIdx++; /* next call will work on next touch */
 }
}

void TS_GetTouchInfo(uint16_t DeviceAddr, uint32_t touchIdx, uint32_t * pWeight, uint32_t * pArea, uint32_t * pEvent)
{
	volatile uint8_t ucReadData = 0;
	uint8_t regAddressXHigh = 0;
	uint8_t regAddressPWeight = 0;
	uint8_t regAddressPMisc = 0;

	if(touchIdx < ft5336_handle.currActiveTouchNb)
	{
		switch(touchIdx)
		{
			case 0 :
				regAddressXHigh = FT5336_P1_XH_REG;
				regAddressPWeight = FT5336_P1_WEIGHT_REG;
				regAddressPMisc = FT5336_P1_MISC_REG;
				break;
			case 1 :
				regAddressXHigh = FT5336_P2_XH_REG;
				regAddressPWeight = FT5336_P2_WEIGHT_REG;
				regAddressPMisc = FT5336_P2_MISC_REG;
				break;
			case 2 :
				regAddressXHigh = FT5336_P3_XH_REG;
				regAddressPWeight = FT5336_P3_WEIGHT_REG;
				regAddressPMisc = FT5336_P3_MISC_REG;
				break;
			case 3 :
				regAddressXHigh = FT5336_P4_XH_REG;
				regAddressPWeight = FT5336_P4_WEIGHT_REG;
				regAddressPMisc = FT5336_P4_MISC_REG;
				break;
			case 4 :
				regAddressXHigh = FT5336_P5_XH_REG;
				regAddressPWeight = FT5336_P5_WEIGHT_REG;
				regAddressPMisc = FT5336_P5_MISC_REG;
				break;
			case 5 :
				regAddressXHigh = FT5336_P6_XH_REG;
				regAddressPWeight = FT5336_P6_WEIGHT_REG;
				regAddressPMisc = FT5336_P6_MISC_REG;
				break;
			case 6 :
				regAddressXHigh = FT5336_P7_XH_REG;
				regAddressPWeight = FT5336_P7_WEIGHT_REG;
				regAddressPMisc = FT5336_P7_MISC_REG;
				break;
			case 7 :
				regAddressXHigh = FT5336_P8_XH_REG;
				regAddressPWeight = FT5336_P8_WEIGHT_REG;
				regAddressPMisc = FT5336_P8_MISC_REG;
				break;
			case 8 :
				regAddressXHigh = FT5336_P9_XH_REG;
				regAddressPWeight = FT5336_P9_WEIGHT_REG;
				regAddressPMisc = FT5336_P9_MISC_REG;
				break;
			case 9 :
				regAddressXHigh = FT5336_P10_XH_REG;
				regAddressPWeight = FT5336_P10_WEIGHT_REG;
				regAddressPMisc = FT5336_P10_MISC_REG;
				break;
			default :
				break;
		}

		ucReadData = TouchIORead(DeviceAddr, regAddressXHigh);
		*pEvent = (ucReadData & FT5336_TOUCH_EVT_FLAG_MASK) >> FT5336_TOUCH_EVT_FLAG_SHIFT;
		ucReadData = TouchIORead(DeviceAddr, regAddressPWeight);
		*pWeight = (ucReadData & FT5336_TOUCH_WEIGHT_MASK) >> FT5336_TOUCH_WEIGHT_SHIFT;
		ucReadData = TouchIORead(DeviceAddr, regAddressPMisc);
		*pArea = (ucReadData & FT5336_TOUCH_AREA_MASK) >> FT5336_TOUCH_AREA_SHIFT;
	}
}

uint8_t TS_Get_GestureId(TS_StateTypeDef *TS_State)
{
	uint32_t gestureId = 0;
	uint8_t ts_status = TS_OK;
	ft5336_TS_GetGestureID(TS_I2C_ADDRESS, &gestureId);
	switch(gestureId)
	{
		case FT5336_GEST_ID_NO_GESTURE :
			TS_State->gestureId = GEST_ID_NO_GESTURE;
			break;
		case FT5336_GEST_ID_MOVE_UP :
			TS_State->gestureId = GEST_ID_MOVE_UP;
			break;
		case FT5336_GEST_ID_MOVE_RIGHT :
			TS_State->gestureId = GEST_ID_MOVE_RIGHT;
			break;
		case FT5336_GEST_ID_MOVE_DOWN :
			TS_State->gestureId = GEST_ID_MOVE_DOWN;
			break;
		case FT5336_GEST_ID_MOVE_LEFT :
			TS_State->gestureId = GEST_ID_MOVE_LEFT;
			break;
		case FT5336_GEST_ID_ZOOM_IN :
			TS_State->gestureId = GEST_ID_ZOOM_IN;
			break;
		case FT5336_GEST_ID_ZOOM_OUT :
			TS_State->gestureId = GEST_ID_ZOOM_OUT;
			break;
		default :
			ts_status = TS_ERROR;
			break;
	}

	return(ts_status);
}

void ft5336_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId)
{
	volatile uint8_t ucReadData = 0;
	ucReadData = TouchIORead(DeviceAddr, FT5336_GEST_ID_REG);
	*pGestureId = ucReadData;
}
