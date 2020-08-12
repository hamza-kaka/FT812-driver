/*! 
@file dma
@brief  code for dma functionality in FT812 driver 
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
 */
#ifndef  DMA_h
#define  DMA_h
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_common.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/
#define DMA_MUX DMAMUX0
#define SPI_TX_SRC (39)
#define SPI_RX_SRC (38)

#define DMA_TX_CHNL (14)
#define DMA_RX_CHNL (15)
#define PUSH_REG 0x400AC034
#define POP_REG  0x400AC038
#define TX_BYTES (1)
#define S_OFF    (1) // DMA source address offset in bytes
#define RX_BYTES (1)
#define DMA_MAX_ITER (0x7FFF)

/*******************************************************************************
 * Enums
 ******************************************************************************/

/*******************************************************************************
 * Structs
 ******************************************************************************/

 /*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

void DmaInit();
void InterruptInit();
void DmaInterruptInit();
void DmaReadInit(uint8_t *buffer,uint32_t count);
void DmaWriteInit(uint8_t *buffer,uint32_t count);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

#if defined(__cplusplus)
}
#endif
#endif
/* EOF */
