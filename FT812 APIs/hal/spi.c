/*! 
@file spi
@brief  SPI functionality 
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "spi.h" 
#include "gpu_hal.h"
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

 void ClearFlags()
 {
   SPI2->SR  = SPI_SR_RFDF(1) | SPI_SR_TCF(1) | SPI_SR_TFFF(1) ;
 }
 
uint8_t Read8 ()
{
  uint8_t dummy =0;
	SPI2->PUSHR = SPI_PUSHR_TXDATA(dummy) |  SPI_PUSHR_PCS(1) | SPI_PUSHR_CONT_MASK;  
	while ((SPI2->SR & SPI_SR_RFDF_MASK)==0)
	 {}
		(void)SPI2->POPR; 
		SPI2->SR  = SPI_SR_RFDF(1) | SPI_SR_TCF(1);
	
		 dummy = SPI2->POPR;	 
		 return dummy;
}

void Send8Spi(uint8_t txdata)
{
	SPI2->PUSHR = SPI_PUSHR_TXDATA(txdata) |  SPI_PUSHR_PCS(1) | SPI_PUSHR_CONT_MASK; 
	while ((SPI2->SR & SPI_SR_RFDF_MASK)==0)
	 {}
		 (void)SPI2->POPR;
		SPI2->SR  = SPI_SR_RFDF(1) | SPI_SR_TCF(1);
    		 
}

void Send8Dummy()
{
	uint8_t dummy =0;
	SPI2->PUSHR = SPI_PUSHR_TXDATA(dummy) |  SPI_PUSHR_PCS(1) | SPI_PUSHR_CONT_MASK;  
	while ((SPI2->SR & SPI_SR_RFDF_MASK)==0)
	 {}
		(void)SPI2->POPR; 
		SPI2->SR  = SPI_SR_RFDF(1) | SPI_SR_TCF(1);
}

void CeaseTrans()
{
	SPI2->PUSHR = 0;
		 while ((SPI2->SR & SPI_SR_RFDF_MASK)==0)
	 {}
		 (void)SPI2->POPR;
		SPI2->SR  = SPI_SR_RFDF(1) | SPI_SR_TCF(1); 
}

void SpiInit()
{
	
//  initialize pins and clock
	SIM->SCGC5 |= SIM_SCGC5_PORTD(1);
	
	PORTD->PCR[11] = PORT_PCR_MUX(2);
	PORTD->PCR[12] = PORT_PCR_MUX(2);
	PORTD->PCR[13] = PORT_PCR_MUX(2);
	PORTD->PCR[14] = PORT_PCR_MUX(2);
	
//	SPI clock enable
	clock_ip_name_t const s_dspiClock[] = DSPI_CLOCKS;
	CLOCK_EnableClock(s_dspiClock[DSPI_GetInstance(SPI_MODULE_NO)]);

}

void SpiDeInit()
{
  //  de initialize pins and clock
	SIM->SCGC5 |= SIM_SCGC5_PORTD(0);
	
	PORTD->PCR[11] = PORT_PCR_MUX(2);
	PORTD->PCR[12] = PORT_PCR_MUX(2);
	PORTD->PCR[13] = PORT_PCR_MUX(2);
	PORTD->PCR[14] = PORT_PCR_MUX(2);
	
	SIM->SCGC3 = SIM_SCGC3_SPI2(0);
}

/* EOF */