/*! 
@file image_loading
@brief 
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
 */
#ifndef IMAGE_LOADING_h
#define IMAGE_LOADING_h
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "gpu_hal.h"
/*******************************************************************************
 * Defines
 ******************************************************************************/
 #define IMAGE_LOAD (1)
 
#define RAM_IMAGES_PAN_LUT 0
#define RAM_IMAGES_PAN 512

#define RAM_IMAGES_DOWNLOAD_LUT 34312
#define RAM_IMAGES_DOWNLOAD 34824

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

void UploadImages(Ft_Gpu_Hal_Context_t* phost);
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