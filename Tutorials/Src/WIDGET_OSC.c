#include <WIDGET_OSC.h>
#include "GUI.h"
#include "WM.h"
#include "DIALOG.h"
#include "SDRAM_MEMORY.h"

#define DISPLAY_BUFFER_ADDR MEM_OSC_DISP_BUFFER
#define DISPLAY_BUFFER_ADDR_SIZE MEM_OSC_DISP_BUFFER_SIZE

//int DefaultTrig(OscilloscopeObj OscObj);

OscilloscopeObj OSC_Obj = {
	(void *) DISPLAY_BUFFER_ADDR,
	{360, 240},
	{0,0},
//	{DefaultTrig, ChannelA, 0, 0},
	{ChannelA, 0, 0},
	{	{0, (void *)MEM_CHANNEL_1, MEM_CHANNEL_SIZE, 0, 0, 0},
		{0, (void *)MEM_CHANNEL_2, MEM_CHANNEL_SIZE, 0, 0, 0}
	},
	{0,0},
	{{0,0,0}, {0,0}, {0,0}}
};

/* MYWIDGET_Callback */
void OSC_Callback(WM_MESSAGE * pMsg) {

	WM_HWIN hWin;
	GUI_RECT WinRect;

	hWin = pMsg->hWin;
	WM_GetWindowRectEx(hWin, &WinRect);
	GUI_MoveRect(&WinRect, -WinRect.x0, -WinRect.y0);

	switch (pMsg->MsgId)
	{
		case WM_PAINT:
		{
			/* Drawing background */
			GUI_COLOR backColor = GUI_BLACK;
			GUI_SetColor(backColor);
			GUI_ClearRect(0, 0, OSC_Obj.size.SizeX, OSC_Obj.size.SizeY);

			/* Drawing display grid */

			uint8_t hor = 8;		//grid 12x8
			uint8_t vert = 12;

			uint8_t pxHor = OSC_Obj.size.SizeX / vert;		//pixels per cell
			uint8_t pxVert = OSC_Obj.size.SizeY / hor;

			uint16_t startX = 0;	//start position for a draw
			uint16_t startY = 0;

			GUI_COLOR gridColor = GUI_DARKGREEN;
			GUI_SetColor(gridColor);

			for(uint8_t i = 0; i <= hor; i++)
			{
				GUI_DrawHLine(startY+(i*pxHor), startX, OSC_Obj.size.SizeX);
			}

			for(uint8_t i = 0; i <= vert; i++)
			{
				GUI_DrawVLine(startX+(i*pxVert), startY, OSC_Obj.size.SizeY);
			}

			break;
		}
		default:
		{
			WM_DefaultProc(pMsg);
		}
	}
}

/* WIDGET Create */

WM_HWIN OSC_Create(int x0, int y0, int xSize, int ySize, WM_HWIN hWinParent)
{
	WM_HWIN hWin;

	OSC_Obj.size.SizeX = xSize;
	OSC_Obj.size.SizeY = ySize;

    hWin = WM_CreateWindowAsChild(x0, y0, xSize, ySize, hWinParent, 0, OSC_Callback, 0);
    return hWin;
}

WM_HWIN OSC_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, WM_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb)
{
	WM_HWIN hWin;

	OSC_Obj.size.SizeX = pCreateInfo->xSize;
	OSC_Obj.size.SizeY = pCreateInfo->ySize;

    hWin = WM_CreateWindowAsChild(x0, y0, OSC_Obj.size.SizeX, OSC_Obj.size.SizeY, hWinParent, 0, OSC_Callback, 0);
    return hWin;
}
