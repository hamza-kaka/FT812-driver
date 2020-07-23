/*! 
@file gpu_hal
@brief  HAL for SPI, DMA's, and other functions
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
 */
#ifndef  FT_GPU_HAL_H
#define  FT_GPU_HAL_H
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_dspi.h"
#include "datatypes_ft81x.h"
#include "fsl_common.h"
#include "ft81xdef.h"
#include "fsl_dmamux.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define def_SPI_config (1) //default SPI config

#define FT_DispPIN (0x80)

#define TR1 0x00003FE7    //touch transform definitions
#define TR2 0xFFFF6B9F
#define TR3 0x02166707
#define TR4 0xFFFFDF7B
#define TR5 0xFFFF6FFD
#define TR6 0x021EA8F3

#define SPI_MODULE_NO (SPI2)
#define CONT_CLCK_ENABLE (0)
#define CONT_PCS_ENABLE (1)
#define DEF_PCS_NO (0)

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

#define BUFFER_OPTIMIZATION (1)

#define FT_GPU_81X_RESET_ACTIVE 0x000268
#define FT_GPU_81X_RESET_REMOVAL 0x002068

#define FT_GPU_CORE_RESET  (0x68)

#define FT_SPI_ONE_DUMMY_BYTE	(0x00)
#define FT_SPI_TWO_DUMMY_BYTE	(0x04)
#define FT_SPI_SINGLE_CHANNEL	(0x00)
#define FT_SPI_DUAL_CHANNEL		(0x01)
#define FT_SPI_QUAD_CHANNEL		(0x02)

/*******************************************************************************
 * Enums
 ******************************************************************************/
typedef enum {
	FT_GPU_HAL_OPENED,
	FT_GPU_HAL_READING,
	FT_GPU_HAL_WRITING,
	FT_GPU_HAL_CLOSED,

	FT_GPU_HAL_STATUS_COUNT,
	FT_GPU_HAL_STATUS_ERROR = FT_GPU_HAL_STATUS_COUNT
}FT_GPU_HAL_STATUS_E;


typedef enum {
	FT_GPU_READ = 0,
	FT_GPU_WRITE,
}FT_GPU_TRANSFERDIR_T;

typedef enum {
	FT_GPU_INTERNAL_OSC = 0x48, //default
	FT_GPU_EXTERNAL_OSC = 0x44,
}FT_GPU_PLL_SOURCE_T;


typedef enum {
	FT_GPU_PLL_48M = 0x62,  //default
	FT_GPU_PLL_36M = 0x61,
	FT_GPU_PLL_24M = 0x64,
}FT_GPU_PLL_FREQ_T;

typedef enum {
	FT_GPU_ACTIVE_M =  0x00,  
	FT_GPU_STANDBY_M = 0x41,//default
	FT_GPU_SLEEP_M =   0x42,
	FT_GPU_POWERDOWN_M = 0x50,
}FT_GPU_POWER_MODE_T;


#ifdef FT_81X_ENABLE
	typedef enum {
		FT_GPU_SYSCLK_DEFAULT = 0x00,  //default 60mhz
		FT_GPU_SYSCLK_72M = 0x06, 
		FT_GPU_SYSCLK_60M = 0x05,  
		FT_GPU_SYSCLK_48M = 0x04,  
		FT_GPU_SYSCLK_36M = 0x03,
		FT_GPU_SYSCLK_24M = 0x02,
	}FT_GPU_81X_PLL_FREQ_T;

	typedef enum{
		FT_GPU_MAIN_ROM = 0x80,  //main graphicas ROM used 
		FT_GPU_RCOSATAN_ROM = 0x40,  //line slope table used for 
		FT_GPU_SAMPLE_ROM = 0x20,  //JA samples
		FT_GPU_JABOOT_ROM = 0x10, //JA microcode
		FT_GPU_J1BOOT_ROM = 0x08, //J1 microcode
		FT_GPU_ADC = 0x01,  //
		FT_GPU_POWER_ON_ROM_AND_ADC = 0x00,  //specify this element to power on all ROMs and ADCs
	}FT_GPU_81X_ROM_AND_ADC_T;

	typedef enum{
		FT_GPU_5MA = 0x00,  //default current
		FT_GPU_10MA = 0x01,
		FT_GPU_15MA = 0x02,
		FT_GPU_20MA = 0x03,
	}FT_GPU_81X_GPIO_DRIVE_STRENGTH_T;

	typedef enum{
		FT_GPU_GPIO0 = 0x00,
		FT_GPU_GPIO1 = 0x04,
		FT_GPU_GPIO2 = 0x08,
		FT_GPU_GPIO3 = 0x0C,
		FT_GPU_GPIO4 = 0x10,
		FT_GPU_DISP = 0x20,
		FT_GPU_DE = 0x24,
		FT_GPU_VSYNC_HSYNC = 0x28,
		FT_GPU_PCLK = 0x2C,
		FT_GPU_BACKLIGHT = 0x30,
		FT_GPU_R_G_B = 0x34,
		FT_GPU_AUDIO_L = 0x38,
		FT_GPU_INT_N = 0x3C,
		FT_GPU_TOUCHWAKE = 0x40,
		FT_GPU_SCL = 0x44,
		FT_GPU_SDA = 0x48,
		FT_GPU_SPI_MISO_MOSI_IO2_IO3 = 0x4C,
	}FT_GPU_81X_GPIO_GROUP_T;


#endif
	
	/* Enums for number of SPI dummy bytes and number of channels */
typedef enum {
	FT_GPU_SPI_SINGLE_CHANNEL = 0,
	FT_GPU_SPI_DUAL_CHANNEL = 1,
	FT_GPU_SPI_QUAD_CHANNEL = 2,
}FT_GPU_SPI_NUMCHANNELS_T;
typedef enum {
	FT_GPU_SPI_ONEDUMMY = 1,
	FT_GPU_SPI_TWODUMMY = 2,
}FT_GPU_SPI_NUMDUMMYBYTES;


/*******************************************************************************
 * Structs
 ******************************************************************************/
typedef struct {
	unsigned char pcsNo;
	unsigned char contClock;
	unsigned char contPcs;
	
		ft_uint8_t spiCsPinNo;		//spi chip select number of ft8xx chip
	
		ft_uint16_t spiClckRtKhz;  //In KHz

	  ft_uint8_t pdnPinNo;				//ft8xx power down pin number
	
}Ft_Gpu_Hal_Config_t;


typedef struct {	
	SPI_Type* spiModule;
}Ft_Gpu_HalInit_t;


typedef struct {	
	ft_uint32_t length; //IN and OUT
	ft_uint32_t address;
	ft_uint8_t  *buffer;
}Ft_Gpu_App_Transfer_t;


typedef struct {
//	Ft_Gpu_App_Context_t    app_header;
	Ft_Gpu_Hal_Config_t     hal_config;

    ft_uint16_t 			ft_cmd_fifo_wp; //coprocessor fifo write pointer
    ft_uint16_t 			ft_dl_buff_wp;  //display command memory write pointer

	FT_GPU_HAL_STATUS_E 	status;        //OUT
	ft_void_t*          	hal_handle;    //IN/OUT
    ft_void_t*          	hal_handle2;   //IN/OUT LibFT4222 uses this member to store GPIO handle	
	/* Additions specific to ft81x */
//	ft_uint8_t				spichannel;			//variable to contain single/dual/quad channels
	ft_uint8_t				spinumdummy;		//number of dummy bytes as 1 or 2 for spi read
    ft_uint8_t *            spiwrbuf_ptr;
}Ft_Gpu_Hal_Context_t;


/* FIFO buffer management */
typedef struct Ft_Fifo_t{
	ft_uint32_t 		fifo_buff; 	//fifo buffer address
	ft_int32_t 			fifo_len; 	//fifo length
	ft_int32_t 			fifo_wp; 	//fifo write pointer - maintained by host
	ft_int32_t 			fifo_rp;  	//fifo read point - maintained by device

	/* FT800 series specific registers */
	ft_uint32_t 		HW_Read_Reg;	//hardware fifo read register
	ft_uint32_t 		HW_Write_Reg;	//hardware fifo write register
}Ft_Fifo_t;

 /*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

ft_bool_t  Ft_Gpu_Hal_Init(Ft_Gpu_HalInit_t *halinit);
ft_bool_t Ft_Gpu_Hal_Open(Ft_Gpu_Hal_Context_t *host, Ft_Gpu_HalInit_t *halmodule);

/*The APIs for reading/writing transfer continuously only with small buffer system*/
ft_void_t               Ft_Gpu_Hal_StartTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw,ft_uint32_t addr);
ft_uint8_t              Ft_Gpu_Hal_Transfer8(Ft_Gpu_Hal_Context_t *host,ft_uint8_t value);
ft_uint16_t             Ft_Gpu_Hal_Transfer16(Ft_Gpu_Hal_Context_t *host,ft_uint16_t value);
ft_uint32_t             Ft_Gpu_Hal_Transfer32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t value);
ft_void_t               Ft_Gpu_Hal_EndTransfer(Ft_Gpu_Hal_Context_t *host);

/*Close and deinit apis*/
ft_void_t              Ft_Gpu_Hal_Close(Ft_Gpu_Hal_Context_t *host, Ft_Gpu_HalInit_t *halmodule);
ft_void_t              Ft_Gpu_Hal_DeInit();

/*Helper function APIs Read*/
ft_uint8_t  Ft_Gpu_Hal_Rd8(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr);
ft_uint16_t Ft_Gpu_Hal_Rd16(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr);
ft_uint32_t Ft_Gpu_Hal_Rd32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr);

/*Helper function APIs Write*/
ft_void_t Ft_Gpu_Hal_Wr8(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint8_t v);
ft_void_t Ft_Gpu_Hal_Wr16(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint16_t v);
ft_void_t Ft_Gpu_Hal_Wr32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint32_t v);

/*APIs for coprocessor Fifo read/write and space management*/
ft_void_t Ft_Gpu_Hal_Updatecmdfifo(Ft_Gpu_Hal_Context_t *host,ft_uint32_t count);
ft_void_t Ft_Gpu_Hal_WrCmd32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t cmd);
ft_void_t Ft_Gpu_Hal_WrCmdBuf(Ft_Gpu_Hal_Context_t *host,ft_uint8_t *buffer,ft_uint32_t count);
ft_void_t Ft_Gpu_Hal_WaitCmdfifo_empty(Ft_Gpu_Hal_Context_t *host);
ft_void_t Ft_Gpu_Hal_ResetCmdFifo(Ft_Gpu_Hal_Context_t *host);
ft_void_t Ft_Gpu_Hal_CheckCmdBuffer(Ft_Gpu_Hal_Context_t *host,ft_uint32_t count);
ft_void_t Ft_Gpu_Hal_ResetDLBuffer(Ft_Gpu_Hal_Context_t *host);
ft_void_t  Ft_Gpu_Hal_StartCmdTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw, ft_uint16_t count);
ft_void_t Ft_Gpu_Hal_Powercycle(Ft_Gpu_Hal_Context_t *host,ft_bool_t up);

/* APIs related to fifo buffer management */
/* API to update the read pointer into the structure */
ft_void_t Ft_Fifo_Init(Ft_Fifo_t *pFifo,ft_uint32_t StartAddress,ft_uint32_t Length,ft_uint32_t HWReadRegAddress,ft_uint32_t HWWriteRegAddress);//Init all the parameters of fifo buffer
ft_void_t Ft_Fifo_Update(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo);//update both the read and write pointers
ft_uint32_t Ft_Fifo_Write(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo,ft_uint8_t *buffer,ft_uint32_t NumbytetoWrite);//just write and update the write register
ft_void_t Ft_Fifo_Write32(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo,ft_uint32_t WriteWord);//just write one word and update the write register
ft_void_t Ft_Fifo_WriteWait(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo,ft_uint8_t *buffer,ft_uint32_t Numbyte);//write and wait for the fifo to be empty
ft_uint32_t Ft_Fifo_GetFreeSpace(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo);//get the free space in the fifo - make sure the return value is maximum of (LENGTH - 4)


void ActivateFT81x(Ft_Gpu_Hal_Context_t* phost,Ft_Gpu_HalInit_t* Lcd_spiModule,Ft_Gpu_Hal_Config_t* Lcd_Spi_def_config);
ft_void_t TouchRegCalib (Ft_Gpu_Hal_Context_t *host);
ft_int32_t hal_strlen(const ft_char8_t *s);
ft_void_t Ft_Gpu_Hal_Sleep(ft_uint32_t ms);

ft_void_t Ft_App_WrCoCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd);
ft_void_t Ft_App_WrDlCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd);
ft_void_t Ft_App_WrCoStr_Buffer(Ft_Gpu_Hal_Context_t *phost,const ft_char8_t *s);
ft_void_t Ft_App_Flush_DL_Buffer(Ft_Gpu_Hal_Context_t *phost);
ft_void_t Ft_App_Flush_Co_Buffer(Ft_Gpu_Hal_Context_t *phost);


ft_void_t Ft_Gpu_ClockSelect(Ft_Gpu_Hal_Context_t *host,FT_GPU_PLL_SOURCE_T pllsource);
ft_void_t Ft_Gpu_PLL_FreqSelect(Ft_Gpu_Hal_Context_t *host,FT_GPU_PLL_FREQ_T freq);
ft_void_t Ft_Gpu_PowerModeSwitch(Ft_Gpu_Hal_Context_t *host,FT_GPU_POWER_MODE_T pwrmode);
ft_void_t Ft_Gpu_CoreReset(Ft_Gpu_Hal_Context_t *host);
ft_void_t Ft_Gpu_Hal_StartTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw,ft_uint32_t addr);
ft_void_t Ft_Gpu_Hal_WrMem(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr,ft_uint8_t *buffer, ft_uint32_t length);
ft_void_t Ft_Gpu_Hal_WrMemFromFlash(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_prog_uchar8_t *buffer, ft_uint32_t length);
ft_void_t Ft_Gpu_Hal_WrCmdBufFromFlash(Ft_Gpu_Hal_Context_t *host,FT_PROGMEM ft_prog_uchar8_t *buffer,ft_uint32_t count);
ft_void_t Ft_Gpu_Hal_RdMem(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint8_t *buffer, ft_uint32_t length);
ft_void_t Ft_Gpu_Hal_WaitLogo_Finish(Ft_Gpu_Hal_Context_t *host);
ft_uint8_t Ft_Gpu_Hal_TransferString(Ft_Gpu_Hal_Context_t *host,const ft_char8_t *string);
ft_void_t Ft_Gpu_HostCommand(Ft_Gpu_Hal_Context_t *host,ft_uint8_t cmd);
ft_void_t Ft_Gpu_HostCommand_Ext3(Ft_Gpu_Hal_Context_t *host,ft_uint32_t cmd);
ft_int32_t Ft_Gpu_Hal_Dec2Ascii(ft_char8_t *pSrc,ft_int32_t value);
ft_uint8_t Ft_Gpu_Hal_WaitCmdfifo_empty_status(Ft_Gpu_Hal_Context_t *host);
ft_void_t Ft_Gpu_Hal_WrCmdBuf_nowait(Ft_Gpu_Hal_Context_t *host,ft_uint8_t *buffer,ft_uint32_t count);
ft_uint16_t Ft_Gpu_Cmdfifo_Freespace(Ft_Gpu_Hal_Context_t *host);
ft_int16_t Ft_Gpu_Hal_SetSPI(Ft_Gpu_Hal_Context_t *host,FT_GPU_SPI_NUMCHANNELS_T numchnls,FT_GPU_SPI_NUMDUMMYBYTES numdummy);

#ifdef FT_81X_ENABLE
ft_void_t Ft_Gpu_81X_SelectSysCLK(Ft_Gpu_Hal_Context_t *host, FT_GPU_81X_PLL_FREQ_T freq);
ft_void_t Ft_GPU_81X_PowerOffComponents(Ft_Gpu_Hal_Context_t *host, ft_uint8_t val);
ft_void_t Ft_GPU_81X_PadDriveStrength(Ft_Gpu_Hal_Context_t *host, FT_GPU_81X_GPIO_DRIVE_STRENGTH_T strength, FT_GPU_81X_GPIO_GROUP_T group);
ft_void_t Ft_Gpu_81X_ResetActive(Ft_Gpu_Hal_Context_t *host);
ft_void_t Ft_Gpu_81X_ResetRemoval(Ft_Gpu_Hal_Context_t *host);
#endif

//ft_uint32_t Ft_Gpu_CurrentFrequency(Ft_Gpu_Hal_Context_t *host);
//ft_int32_t Ft_Gpu_ClockTrimming(Ft_Gpu_Hal_Context_t *host,ft_int32_t LowFreq);

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




