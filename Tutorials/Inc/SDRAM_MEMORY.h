#define MEM_START (0xC0000000)

#define MEM_LTDC_LAYER_1 		MEM_START
#define MEM_LTDC_LAYER_1_SIZE 	(480*272*4)

#define MEM_LTDC_LAYER_2 		(MEM_LTDC_LAYER_1 + MEM_LTDC_LAYER_1_SIZE)
#define MEM_LTDC_LAYER_2_SIZE 	(480*272*2)

//---------------------------------------
/* RESERVED MEMORY, OTHERWISE EMWIN DOESN'T WORK */
#define MEM_RESERVED			(480*272*4)
//---------------------------------------

#define MEM_EMWIN				(MEM_LTDC_LAYER_2 + MEM_LTDC_LAYER_2_SIZE + MEM_RESERVED)
#define MEM_EMWIN_SIZE			(15*1024)

#define MEM_OSC_DISP_BUFFER		(MEM_EMWIN + MEM_EMWIN_SIZE)
#define MEM_OSC_DISP_BUFFER_SIZE (360)
								//next free address is 0xC0142968
								//or (MEM_OSC_DISP_BUFFER + MEM_OSC_DISP_BUFFER_SIZE)
//... some free ram ...

#define MEM_CHANNEL_SIZE		(3*1024*1024)

#define MEM_CHANNEL_1			(0xC0200000)	//3Mbytes per channel
#define MEM_CHANNEL_2			(0xC0500000)
