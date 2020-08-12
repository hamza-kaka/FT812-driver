/*! 
@file dma
@brief  code for dma functionality in FT812 driver 
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "Dma.h"
#include "fsl_common.h"
#include "fsl_dmamux.h"

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
  extern volatile bool dmaDone=0;		
	
/*******************************************************************************
 * Code
 ******************************************************************************/
 
 void DMA15_DMA31_IRQHandler(void)
{
	DMA0->CINT = DMA_CINT_CINT(15);
	dmaDone = 1;
} 


void DMA14_DMA30_IRQHandler(void)
{
	DMA0->CINT = DMA_CINT_CINT(14);
//	dmaDone = 1;
} 


void DmaWriteInit(uint8_t *buffer,uint32_t count)
{
	
	uint8_t dmaBuff =0;
  DMA0->TCD[DMA_TX_CHNL].NBYTES_MLNO = TX_BYTES;
	DMA0->TCD[DMA_TX_CHNL].SOFF = S_OFF;
	DMA0->TCD[DMA_TX_CHNL].DADDR = PUSH_REG;
  DMA0->TCD[DMA_TX_CHNL].ATTR = DMA_ATTR_DSIZE(0) | DMA_ATTR_SSIZE(0);
  DMA0->TCD[DMA_TX_CHNL].DOFF = 0;
  DMA0->TCD[DMA_TX_CHNL].DLAST_SGA = 0;
  DMA0->TCD[DMA_TX_CHNL].SLAST = 0;
 	DMA0->TCD[DMA_TX_CHNL].CSR =  DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ(1);
	DMA0->TCD[DMA_TX_CHNL].BITER_ELINKNO = DMA0->TCD[DMA_TX_CHNL].CITER_ELINKNO = count;
	DMA0->TCD[DMA_TX_CHNL].SADDR = (uint32_t)buffer;
	
		
	
	DMA0->TCD[DMA_RX_CHNL].NBYTES_MLNO = RX_BYTES;
	DMA0->TCD[DMA_RX_CHNL].SOFF = 0;
	DMA0->TCD[DMA_RX_CHNL].CSR =  DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ(1);
	DMA0->TCD[DMA_RX_CHNL].SADDR = POP_REG;
  DMA0->TCD[DMA_RX_CHNL].ATTR = DMA_ATTR_DSIZE(0) | DMA_ATTR_SSIZE(0);
  DMA0->TCD[DMA_RX_CHNL].DOFF = 0;
  DMA0->TCD[DMA_RX_CHNL].DLAST_SGA = 0;
  DMA0->TCD[DMA_RX_CHNL].SLAST = 0;
 	DMA0->TCD[DMA_RX_CHNL].DADDR = (uint32_t)&dmaBuff;
	DMA0->TCD[DMA_RX_CHNL].BITER_ELINKNO = DMA0->TCD[DMA_RX_CHNL].CITER_ELINKNO = count;
	
	
	DMA0->ERQ = (uint32_t)(1<<DMA_TX_CHNL | 1<<DMA_RX_CHNL);
}

void DmaReadInit(uint8_t *buffer,uint32_t count)
{

  uint8_t dmaBuff =0;
  DMA0->TCD[DMA_TX_CHNL].NBYTES_MLNO = TX_BYTES;
	DMA0->TCD[DMA_TX_CHNL].SOFF = 0;
	DMA0->TCD[DMA_TX_CHNL].DADDR = PUSH_REG;
  DMA0->TCD[DMA_TX_CHNL].ATTR = DMA_ATTR_DSIZE(0) | DMA_ATTR_SSIZE(0);
  DMA0->TCD[DMA_TX_CHNL].DOFF = 0;
  DMA0->TCD[DMA_TX_CHNL].DLAST_SGA = 0;
  DMA0->TCD[DMA_TX_CHNL].SLAST = 0;
 	DMA0->TCD[DMA_TX_CHNL].CSR =  DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ(1);
	DMA0->TCD[DMA_TX_CHNL].BITER_ELINKNO = DMA0->TCD[DMA_TX_CHNL].CITER_ELINKNO = count;
	DMA0->TCD[DMA_TX_CHNL].SADDR = (uint32_t)&dmaBuff;
	
	DMA0->TCD[DMA_RX_CHNL].NBYTES_MLNO = RX_BYTES;
	DMA0->TCD[DMA_RX_CHNL].SOFF = 0;
	DMA0->TCD[DMA_RX_CHNL].CSR =  DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ(1);
	DMA0->TCD[DMA_RX_CHNL].SADDR = POP_REG;
  DMA0->TCD[DMA_RX_CHNL].ATTR = DMA_ATTR_DSIZE(0) | DMA_ATTR_SSIZE(0);
  DMA0->TCD[DMA_RX_CHNL].DOFF = S_OFF;
  DMA0->TCD[DMA_RX_CHNL].DLAST_SGA = 0;
  DMA0->TCD[DMA_RX_CHNL].SLAST = 0;
	DMA0->TCD[DMA_RX_CHNL].DADDR = (uint32_t)buffer;
	DMA0->TCD[DMA_RX_CHNL].BITER_ELINKNO = DMA0->TCD[DMA_RX_CHNL].CITER_ELINKNO = count;
	
	  DMA0->ERQ = (uint32_t)(1<<DMA_TX_CHNL | 1<<DMA_RX_CHNL);
		
}

void DmaInterruptInit()
{
	NVIC_EnableIRQ(DMA15_DMA31_IRQn); 
	NVIC_EnableIRQ(DMA14_DMA30_IRQn);
}

void DmaInit()
{

	//	DMA MUX initiazation
	DMAMUX_Init(DMAMUX0);
  DMAMUX_SetSource(DMAMUX0,DMA_RX_CHNL,SPI_RX_SRC);
  DMAMUX_SetSource(DMAMUX0,DMA_TX_CHNL,SPI_TX_SRC);
  DMAMUX_EnableChannel(DMAMUX0,DMA_TX_CHNL);
  DMAMUX_EnableChannel(DMAMUX0,DMA_RX_CHNL);
	
//	DMA clock enable and initialization
  SIM->SCGC7 = SIM_SCGC7_DMA_MASK;
	
	
if(DMA_TX_CHNL>15 && DMA_RX_CHNL>15)
	DMA0->CR =   DMA_CR_GRP1PRI(1);
else if(DMA_TX_CHNL<=15 && DMA_RX_CHNL<=15)
	DMA0->CR =   DMA_CR_GRP0PRI(1);

  DmaInterruptInit();
}


/* EOF */