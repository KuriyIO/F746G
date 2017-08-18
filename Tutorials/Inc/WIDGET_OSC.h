#ifndef WIDGET_OSC_H_
#define WIDGET_OSC_H_

#include "stdint.h"
#include "GUI.h"
#include "DIALOG.h"
#include "WM.h"

typedef enum
{
	ChannelA,
	ChannelB
} OscSourceChannel;

typedef struct
{
//    int (*trigFunc)(OscilloscopeSettings);
	OscSourceChannel chan;
	uint32_t param1;
	uint32_t param2;
} OscTriggerTypeDef;

typedef struct
{
	uint8_t isEnable;
	void *BufferAddr;
	uint32_t bufferSize;
	uint8_t invertFlag;
	uint32_t param1;
	uint32_t param2;
} OscChannelTypeDef;

typedef struct
{
	uint32_t param1;
	uint32_t param2;
} OscHorizontalTypeDef;

typedef struct
{
	uint32_t pos1;
	uint32_t pos2;
} OscCursorTypeDef;

typedef struct
{
	unsigned HorEnable: 1;
	unsigned VertEnable: 1;
	unsigned VoltsEnable: 1;
} OscMeasureStatusTypeDef;

typedef struct
{
	OscMeasureStatusTypeDef status;
	OscCursorTypeDef horizontal;
	OscCursorTypeDef vertical;
} OscMeasureTypeDef;

typedef struct
{
	uint16_t SizeX;
	uint16_t SizeY;
} OscDispaySizeTypeDef;

typedef struct
{
	unsigned TrigEnabled: 	1;
	unsigned RecordEnabled: 1;
} OscStatusTypeDef;

typedef struct
{
	void *DisplayBufferAddr;
	OscDispaySizeTypeDef size;
	OscStatusTypeDef status;
	OscTriggerTypeDef trig;
	OscChannelTypeDef Ch[2];
	OscHorizontalTypeDef horizontal;
	OscMeasureTypeDef measure;
} OscilloscopeObj;

WM_HWIN OSC_Create(int x0, int y0, int xSize, int ySize, WM_HWIN hWinParent);
WM_HWIN OSC_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, WM_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

#endif /* WIDGET_OSC_H_ */
