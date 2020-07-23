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
#include "fsl_common.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

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
																									///////////////////////////////////////////// DMA stuff ///////////////////////////////////////////////////
extern volatile bool dmaDone;
void DMA15_DMA31_IRQHandler(void)
{
	DMA0->CINT = DMA_CINT_CINT(15);
	dmaDone = 1;
} 

void DMA14_DMA30_IRQHandler(void)
{
	DMA0->CINT = DMA_CINT_CINT(14);
	dmaDone = 1;
} 

																											////////////////////////////////////////////////////////////////////////////////////////////////



int main()
{
  BOARD_InitBootClocks();
	
	
																												///////////////////////// DMA stuff//////////////////////////////   

	NVIC_EnableIRQ(DMA15_DMA31_IRQn); 
	NVIC_EnableIRQ(DMA14_DMA30_IRQn);
	
	
																											///////////////////////////////////////////////////////////////////
	

	
	Ft_Gpu_Hal_Config_t Lcd_Spi_def_config = {DEF_PCS_NO, CONT_CLCK_ENABLE, CONT_PCS_ENABLE };
	Ft_Gpu_Hal_Context_t Lcd_Spi_Handler;
	Ft_Gpu_HalInit_t Lcd_spiModule;
	Ft_Gpu_Hal_Context_t* phost = &Lcd_Spi_Handler;	
	
	ActivateFT81x(phost,&(Lcd_spiModule),&Lcd_Spi_def_config);
	
																								////////////////////////////// display list and command buffer code /////////////////////////////////////////
	
	


}

/* EOF */








