/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.32                          *
*        Compiled Oct  8 2015, 11:59:02                              *
*        (c) 2015 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
#include "DIALOG.h"
WM_HWIN CreateHomePage(void);
WM_HWIN CreateChAPage(void);
WM_HWIN CreateChBPage(void);
WM_HWIN CreateHorizontalPage(void);
WM_HWIN CreateMeasurePage(void);
WM_HWIN CreateTriggerPage(void);
// USER END

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0        (GUI_ID_USER + 0x00)



// USER START (Optionally insert additional defines)
#define ID_MULTIPAGE_0        (GUI_ID_USER + 0x01)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "MainWnd", ID_WINDOW_0, 0, 0, 480, 272, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  { MULTIPAGE_CreateIndirect, "Multipage", ID_MULTIPAGE_0, 0, 0, 480, 272, 0, 0x0, 0 }
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'MainWnd'
    //
    hItem = pMsg->hWin;
    WINDOW_SetBkColor(hItem, GUI_MAKE_COLOR(0x00000000));

    // USER START (Optionally insert additional code for further widget initialization)

    hItem = WM_GetDialogItem(pMsg->hWin, ID_MULTIPAGE_0);

    MULTIPAGE_SetBkColor(hItem, 0,1);
    MULTIPAGE_SetFont(hItem, GUI_FONT_20B_1);

    MULTIPAGE_SetTabHeight(hItem, 30);

    MULTIPAGE_AddPage(hItem, CreateHomePage(), "Home");
    MULTIPAGE_AddPage(hItem, CreateTriggerPage(), "Trigger");
    MULTIPAGE_AddPage(hItem, CreateChAPage(), "Ch A");
    MULTIPAGE_AddPage(hItem, CreateChBPage(), "Ch B");
    MULTIPAGE_AddPage(hItem, CreateHorizontalPage(), "Horizontal");
    MULTIPAGE_AddPage(hItem, CreateMeasurePage(), "Measure");

    MULTIPAGE_SelectPage(hItem, 0);

    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {

      // USER START (Optionally insert additional code for further notification handling)

      case ID_MULTIPAGE_0: // Notifications sent by 'Multipage'
	  switch(NCode) {
	  case WM_NOTIFICATION_CLICKED:
		// USER START (Optionally insert code for reacting on notification message)
		  asm("nop");
		// USER END
		break;
	  case WM_NOTIFICATION_RELEASED:
		// USER START (Optionally insert code for reacting on notification message)
		// USER END
		break;
	  case WM_NOTIFICATION_MOVED_OUT:
		// USER START (Optionally insert code for reacting on notification message)
		// USER END
		break;
	  case WM_NOTIFICATION_VALUE_CHANGED:
		// USER START (Optionally insert code for reacting on notification message)
		// USER END
		break;

      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateMainWnd
*/
WM_HWIN CreateMainWnd(void);
WM_HWIN CreateMainWnd(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
