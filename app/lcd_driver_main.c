/*! 
@file lcd_driver_main.c
@brief  main file for FT81x driver 
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "ft81x_copro_cmds.h"
#include "gpu_hal.h"
#include "fsl_common.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "timers.h"
#include "image_loading.h"
#include "screens.h"
#include "spi.h"

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


int main()
{
  BOARD_InitBootClocks();
	BOARD_InitBootPins();
	
	
	

	
	Ft_Gpu_Hal_Config_t Lcd_Spi_def_config = {DEF_PCS_NO, CONT_CLCK_ENABLE, CONT_PCS_ENABLE };
	Ft_Gpu_Hal_Context_t Lcd_Spi_Handler;
	Ft_Gpu_HalInit_t Lcd_spiModule;
	Ft_Gpu_Hal_Context_t* phost = &Lcd_Spi_Handler;	

		
	ActivateFT81x(phost,&(Lcd_spiModule),&Lcd_Spi_def_config);
	
	
																								////////////////////////////// display list and command buffer code /////////////////////////////////////////
											
	
while(1)
{}
}

/* EOF */








