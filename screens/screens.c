/*! 
@file 
@brief 
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "screens.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Enums
 ******************************************************************************/

/*******************************************************************************
 * Structs
 ******************************************************************************/

 /*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
 
 int PollForTag(Ft_Gpu_Hal_Context_t* phost)
{
	int tagNum = 0;
	tagNum = Ft_Gpu_Hal_Rd8(phost, REG_TOUCH_TAG); 
	
	PitSetPeriod(100000);
	PitWaitStart();
	PitStopTimer();
	PitClearFlags();
	
	return tagNum;
}

																																													/***************************************************** sample screens to show screen switching *****************************************/
																																													
void BluePage(Ft_Gpu_Hal_Context_t* phost)
{
	Ft_Gpu_CoCmd_Dlstart(phost);
	Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(31, 63, 127));
	Ft_App_WrCoCmd_Buffer(phost,CLEAR_TAG(0));
	Ft_App_WrCoCmd_Buffer(phost,CLEAR(1, 1, 1));
	Ft_App_WrCoCmd_Buffer(phost,TAG(8));
	Ft_Gpu_CoCmd_Button(phost,318, 123, 120, 36, 27, 0, "red");
	Ft_App_WrCoCmd_Buffer(phost,TAG(42));
	Ft_Gpu_CoCmd_Button(phost,46, 127, 120, 36, 27, 0, "purple");
	
	Ft_App_WrCoCmd_Buffer(phost, DISPLAY());
	Ft_Gpu_CoCmd_Swap(phost);
	Ft_App_Flush_Co_Buffer(phost);
	Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
}

void RedPage(Ft_Gpu_Hal_Context_t* phost)
{
	Ft_Gpu_CoCmd_Dlstart(phost);
	Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(153, 3, 23));
	Ft_App_WrCoCmd_Buffer(phost,CLEAR_TAG(0));
	Ft_App_WrCoCmd_Buffer(phost,CLEAR(1, 1, 1));
	Ft_App_WrCoCmd_Buffer(phost,TAG(6));
	Ft_Gpu_CoCmd_Button(phost,318, 123, 120, 36, 27, 0, "blue");
	Ft_App_WrCoCmd_Buffer(phost,TAG(42));
	Ft_Gpu_CoCmd_Button(phost,46, 127, 120, 36, 27, 0, "purple");
	
	Ft_App_WrCoCmd_Buffer(phost, DISPLAY());
	Ft_Gpu_CoCmd_Swap(phost);
	Ft_App_Flush_Co_Buffer(phost);
	Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
}

void PurplePage(Ft_Gpu_Hal_Context_t* phost)
{
	Ft_Gpu_CoCmd_Dlstart(phost);
	Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(116, 82, 151));
	Ft_App_WrCoCmd_Buffer(phost,CLEAR_TAG(0));
	Ft_App_WrCoCmd_Buffer(phost,CLEAR(1, 1, 1));
	Ft_App_WrCoCmd_Buffer(phost,TAG(8));
	Ft_Gpu_CoCmd_Button(phost,318, 123, 120, 36, 27, 0, "red");
	Ft_App_WrCoCmd_Buffer(phost,TAG(6));
	Ft_Gpu_CoCmd_Button(phost,46, 127, 120, 36, 27, 0, "blue");
	
	Ft_App_WrCoCmd_Buffer(phost, DISPLAY());
	Ft_Gpu_CoCmd_Swap(phost);
	Ft_App_Flush_Co_Buffer(phost);
	Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
}

																																																											/**********************************************************************************************/
																																																													
/* EOF */