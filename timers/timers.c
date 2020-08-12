/*! 
@file timers
@brief 
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "timers.h"
#include "fsl_common.h"
#include "fsl_pit.h"

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
 void PitInit()
{

	pit_config_t pitConfig; 
	PIT_GetDefaultConfig(&pitConfig);
	PIT_Init(PIT, &pitConfig);
	
}

 
void PitSetPeriod(int periodUs)
 {
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(periodUs, PIT_SOURCE_CLOCK));
 }
 
 void PitStartTimer()
 {
	PIT_StartTimer(PIT, kPIT_Chnl_0);
 }
 
 void PitStopTimer()
 {
	PIT_StopTimer(PIT, kPIT_Chnl_0);
 }
 
 void PitClearFlags()
 {
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0,kPIT_TimerFlag);
 }

 
 void PitWaitStart()
 {
	 PIT_StartTimer(PIT, kPIT_Chnl_0);
	while(!PIT_GetStatusFlags(PIT, kPIT_Chnl_0))
	{}
 }
 
 void PitTimer(int time)
 {
	PitSetPeriod(time);
	PitWaitStart();
	PitStopTimer();
	PitClearFlags();
 }
/* EOF */