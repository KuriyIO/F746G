/*********************************************************************
*          Portions COPYRIGHT 2016 STMicroelectronics                *
*          Portions SEGGER Microcontroller GmbH & Co. KG             *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2015  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.32 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to STMicroelectronics International
N.V. a Dutch company with a Swiss branch and its headquarters in Plan-
les-Ouates, Geneva, 39 Chemin du Champ des Filles, Switzerland for the
purposes of creating libraries for ARM Cortex-M-based 32-bit microcon_
troller products commercialized by Licensee only, sublicensed and dis_
tributed under the terms and conditions of the End User License Agree_
ment supplied by STMicroelectronics International N.V.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LCDConf_Lin_Template.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

/**
  ******************************************************************************
  * @attention
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "GUI.h"
#include "main.h"
#include "stm32f7xx_hal.h"
#include "GUIDRV_Lin.h"

extern const GUI_DEVICE_API GUIDRV_Lin_32_API;
#define GUIDRV_LIN_32      &GUIDRV_Lin_32_API

/*********************************************************************
*
*       Layer configuration (to be modified)
*
**********************************************************************
*/
//
// Physical display size
//
#define XSIZE_PHYS 480
#define YSIZE_PHYS 272


//
// Buffers / VScreens
//
#define NUM_BUFFERS  3 // Number of multiple buffers to be used
#define NUM_VSCREENS 1 // Number of virtual screens to be used
#define FRAME_BUFFER_ADDRESS 0xC0200000	// Address in SDRAM

/* Variables */
int32_t bufferIndex = 0;
__IO int32_t pending_buffer = 0;
LTDC_LayerCfgTypeDef pLayerCfg;
LTDC_HandleTypeDef hltdc;
uint8_t charvar[4]; //for debug parameters

/* Function prototypes */
static void CUSTOM_CopyBuffer(int LayerIndex, int IndexSrc, int IndexDst);
static void DMA2D_CopyBuffer(void * pSrc, void * pDst, U32 xSize, U32 ySize, U32 OffLineSrc, U32 OffLineDst);
static void CUSTOM_CopyRect(int LayerIndex, int x0, int y0, int x1, int y1, int xSize, int ySize);
static void CUSTOM_FillRect(int LayerIndex, int x0, int y0, int x1, int y1, U32 PixelIndex);
static void DMA2D_FillBuffer(void * pDst, U32 xSize, U32 ySize, U32 OffLine, U32 ColorIndex);
static void CUSTOM_DrawBitmap32bpp(int LayerIndex, int x, int y, U8 const * p, int xSize, int ySize, int BytesPerLine);


//---------------------------------------------------------------------------------------------------------------------
void LTDC_init(void)
{
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 40;
	hltdc.Init.VerticalSync = 9;
	hltdc.Init.AccumulatedHBP = 53;
	hltdc.Init.AccumulatedVBP = 11;
	hltdc.Init.AccumulatedActiveW = 533;
	hltdc.Init.AccumulatedActiveH = 283;
	hltdc.Init.TotalWidth = 565;
	hltdc.Init.TotalHeigh = 285;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	HAL_LTDC_Init(&hltdc);
	HAL_LTDC_ProgramLineEvent(&hltdc, 0);
	HAL_LTDC_EnableDither(&hltdc);

	/* Assert display enable LCD_DISP pin */
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

	/* Assert backlight LCD_BL_CTRL pin */
	HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
}

void Layer_init()
{
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 480;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 272;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg.Alpha = 255;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	pLayerCfg.ImageWidth = 480;
	pLayerCfg.ImageHeight = 272;
	pLayerCfg.FBStartAdress = (uint32_t) 0xC0200000;
	HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0);
}

static void DMA2D_CopyBuffer(void * pSrc, void * pDst, U32 xSize, U32 ySize, U32 OffLineSrc, U32 OffLineDst)
{
	DMA2D->CR = 0x00000000UL | (1 << 9);
	DMA2D->FGMAR = (U32) pSrc;
	DMA2D->OMAR = (U32) pDst;
	DMA2D->FGOR = OffLineSrc;
	DMA2D->OOR = OffLineDst;
	DMA2D->FGPFCCR = LTDC_PIXEL_FORMAT_ARGB8888;
	DMA2D->NLR = (U32) (xSize << 16) | (U16) ySize;
	DMA2D->CR |= DMA2D_CR_START;
	while (DMA2D->CR & DMA2D_CR_START) {}
}

static void CUSTOM_CopyBuffer(int LayerIndex, int IndexSrc, int IndexDst)
{
	U32 BufferSize, AddrSrc, AddrDst;
	BufferSize = XSIZE_PHYS * YSIZE_PHYS * (LCD_GetBitsPerPixel() >> 3); //in bytes
	AddrSrc = FRAME_BUFFER_ADDRESS + BufferSize * IndexSrc;
	AddrDst = FRAME_BUFFER_ADDRESS + BufferSize * IndexDst;
	DMA2D_CopyBuffer((void *) AddrSrc, (void *) AddrDst, XSIZE_PHYS, YSIZE_PHYS, 0, 0);
	bufferIndex = IndexDst;
}

static void CUSTOM_CopyRect(int LayerIndex, int x0, int y0, int x1, int y1, int xSize, int ySize)
{
	U32 AddrSrc, AddrDst;
	AddrSrc = FRAME_BUFFER_ADDRESS + (y0 * XSIZE_PHYS + x0) * (LCD_GetBitsPerPixel() >> 3);
	AddrDst = FRAME_BUFFER_ADDRESS + (y1 * XSIZE_PHYS + x1) * (LCD_GetBitsPerPixel() >> 3);
	DMA2D_CopyBuffer((void *) AddrSrc, (void *) AddrDst, xSize, ySize, (XSIZE_PHYS - xSize), (XSIZE_PHYS - xSize));
}

static void CUSTOM_DrawBitmap32bpp(int LayerIndex, int x, int y, U8 const * p, int xSize, int ySize, int BytesPerLine)
{
	U32 AddrDst;
	int OffLineSrc, OffLineDst;
	AddrDst = FRAME_BUFFER_ADDRESS + (XSIZE_PHYS * YSIZE_PHYS * (LCD_GetBitsPerPixel() >> 3) * bufferIndex) + (y * XSIZE_PHYS + x) * (LCD_GetBitsPerPixel() >> 3);
	OffLineSrc = (BytesPerLine / 4) - xSize;
	OffLineDst = XSIZE_PHYS - xSize;
	DMA2D_CopyBuffer((void *) p, (void *) AddrDst, xSize, ySize, OffLineSrc, OffLineDst);
}

static void DMA2D_FillBuffer(void * pDst, U32 xSize, U32 ySize, U32 OffLine, U32 ColorIndex)
{
	DMA2D->CR = 0x00030000UL | (1 << 9);
	DMA2D->OCOLR = ColorIndex;
	DMA2D->OMAR = (U32) pDst;
	DMA2D->OOR = OffLine;
	DMA2D->OPFCCR = LTDC_PIXEL_FORMAT_ARGB8888;
	DMA2D->NLR = (U32) (xSize << 16) | (U16) ySize;
	DMA2D->CR |= DMA2D_CR_START;
	while (DMA2D->CR & DMA2D_CR_START) {}
}

static void CUSTOM_FillRect(int LayerIndex, int x0, int y0, int x1, int y1, U32 PixelIndex)
{
	U32 AddrDst;
	int xSize, ySize;
	xSize = x1 - x0 + 1;
	ySize = y1 - y0 + 1;
	AddrDst = FRAME_BUFFER_ADDRESS + (XSIZE_PHYS * YSIZE_PHYS * (LCD_GetBitsPerPixel() >> 3) * bufferIndex) + (y0 * XSIZE_PHYS + x0) * (LCD_GetBitsPerPixel() >> 3);
	DMA2D_FillBuffer((void *) AddrDst, xSize, ySize, XSIZE_PHYS - xSize, PixelIndex);
}

void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *hltdc)
{
	U32 Addr;
	if (pending_buffer >= 0)
	{
		Addr = FRAME_BUFFER_ADDRESS + XSIZE_PHYS * YSIZE_PHYS * pending_buffer * (LCD_GetBitsPerPixel() >> 3);
		__HAL_LTDC_LAYER(hltdc, 0)->CFBAR = Addr;
		__HAL_LTDC_RELOAD_CONFIG(hltdc);
		GUI_MULTIBUF_Confirm(pending_buffer);
		pending_buffer = -1;
	}
	HAL_LTDC_ProgramLineEvent(hltdc, 0); //Define the position of the line interrupt
}
/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*   
*/
void LCD_X_Config(void)
{
	LTDC_init();

	/* Set display driver and color conversion for the 1st layer */
	GUI_DEVICE_CreateAndLink(GUIDRV_LIN_32, GUICC_M8888I, 0, 0);

	/* Sets the size of the virtual display area for the 1st layer*/
	LCD_SetVSizeEx(0, XSIZE_PHYS, YSIZE_PHYS);

	/* Sets the size of the physical display area for the 1st layer*/
	LCD_SetSizeEx(0, XSIZE_PHYS, YSIZE_PHYS);

	/* Sets the address of the VRAM for the 1st layer */
	LCD_SetVRAMAddrEx(0, (void *) FRAME_BUFFER_ADDRESS);

	/* For multiple buffers */
	GUI_MULTIBUF_Config(NUM_BUFFERS);

	/* Set custom functions for several operations */
	LCD_SetDevFunc(0, LCD_DEVFUNC_COPYBUFFER, (void(*)(void)) CUSTOM_CopyBuffer);
	LCD_SetDevFunc(0, LCD_DEVFUNC_COPYRECT, (void(*)(void)) CUSTOM_CopyRect);
	LCD_SetDevFunc(0, LCD_DEVFUNC_FILLRECT, (void(*)(void)) CUSTOM_FillRect);
	LCD_SetDevFunc(0, LCD_DEVFUNC_DRAWBMP_32BPP, (void(*)(void)) CUSTOM_DrawBitmap32bpp);

}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if 
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;
  int xPos, yPos;

  switch (Cmd) {
  case LCD_X_INITCONTROLLER:
  {
	  Layer_init();
	  break;
  }
  case LCD_X_SETORG:
  {
	  HAL_LTDC_SetAddress(&hltdc, FRAME_BUFFER_ADDRESS, 0);
	  break;
  }
  case LCD_X_SHOWBUFFER:
  {
	  pending_buffer = ((LCD_X_SHOWBUFFER_INFO *) pData)->Index;
	  break;
  }
  case LCD_X_ON:
  {
	  __HAL_LTDC_ENABLE(&hltdc);
	  break;
  }
  case LCD_X_OFF:
  {
	  __HAL_LTDC_DISABLE(&hltdc);
	  break;
  }
  case LCD_X_SETSIZE:
  {
	  GUI_GetLayerPosEx(0, &xPos, &yPos);
	  HAL_LTDC_SetWindowPosition(&hltdc, xPos, yPos, 0);
	  break;
  }
  default:
    r = -1;
  }
  return r;
}

/*************************** End of file ****************************/
