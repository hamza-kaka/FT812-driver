/*! 
@file gpu_hal.c
@brief  functions related to HAL for FT81x
@details 

@author Hamza Naeem Kakakhel
@copyright Taraz Technologies Pvt. Ltd.
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "gpu_hal.h"
#include "fsl_dspi.h"

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
 
 
   ////////////////////////////////  hardware layer functions ///////////////////////////
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

void WriteToAdd (int address)
{
 uint8_t addArr[3]={0};
 addArr[2] = (uint8_t)address;
 addArr[1] = (uint8_t)(address >> 8);
 addArr[0] = ((uint8_t)(0x80)) | ( address >> 16);
 
 for(int a=0; a<3;a++)
	 Send8Spi(addArr[a]);
}

void ReadFrmAdd (int address)
{
 uint8_t addArr[3]={0};
 addArr[2] = (uint8_t)address;
 addArr[1] = (uint8_t)(address >> 8);
 addArr[0] = (uint8_t)( address >> 16);
 
 for(int a=0; a<3;a++)
	 Send8Spi(addArr[a]);
}


void RegWrite(int address, int param)
{
 uint8_t paramArr[4] = {0};
 paramArr[0] = (uint8_t)param;
 paramArr[1] = (uint8_t)(param >> 8); 
 paramArr[2] = (uint8_t)(param >> 16); 
 paramArr[3] = (uint8_t)(param >> 24); 
 
 WriteToAdd(address);
 
  for(int a=0; a<4;a++)
	{
	 Send8Spi(paramArr[a]);
  }
	CeaseTrans();	 
}

uint8_t ReadID (int address)
{
	ReadFrmAdd(address);
	uint8_t result = 0;
	
			while(result != 0x7C)
			{
				Send8Dummy();
		    result = (uint8_t)SPI2->POPR; 				
	    }
	CeaseTrans();
	return result;
}

int RegRead (int address)
{
  ReadFrmAdd(address);
	uint8_t shift = 0;
	int result=0;
	
	for(int a=0; a<5;a++)
	{
	 Send8Dummy();
		 if(a!=0)
		 {
			 result |= ((int)(SPI2->POPR))<<shift; 
			 shift +=8;
		 }
	}	 
	CeaseTrans();
	return result;
}


void HostCmd (uint8_t cmd)
 {
	 uint8_t cmdArr[3]={cmd,0,0};
  for(int a=0; a<3;a++)
	{
	 Send8Spi(cmdArr[a]);
  }
	CeaseTrans();
 }

 
 void StartRamDl()
{
  int address = 0x300000;    
	WriteToAdd(address);
}

void AddRamDl(dlCmd)
{
  uint8_t dlCmdArr[4] = {0};
 dlCmdArr[0] = (uint8_t)dlCmd;
 dlCmdArr[1] = (uint8_t)(dlCmd >> 8); 
 dlCmdArr[2] = (uint8_t)(dlCmd >> 16); 
 dlCmdArr[3] = (uint8_t)(dlCmd >> 24); 
	
	for(int a=0; a<4;a++)
	 Send8Spi(dlCmdArr[a]);
	
}

void EndRamDl()
{
  CeaseTrans();
}
	 
  
void StartCmdBuff()
{
  int address = 0x308000;    
	WriteToAdd(address);
}

void AddCmdBuff(buff_cmd)
{
  uint8_t buffCmdArr[4] = {0};
 buffCmdArr[0] = (uint8_t)buff_cmd;
 buffCmdArr[1] = (uint8_t)(buff_cmd >> 8); 
 buffCmdArr[2] = (uint8_t)(buff_cmd >> 16); 
 buffCmdArr[3] = (uint8_t)(buff_cmd >> 24); 
	
	for(int a=0; a<4;a++)
	 Send8Spi(buffCmdArr[a]);
	
}

void EndCmdBuff()
{
  CeaseTrans();
}

void CmdBulkWrite()
{
  int address = REG_CMDB_WRITE;    
	WriteToAdd(address);
}


void SendString (char* str)
{
	uint32_t charCount = 0;
	for(int a=0;;a++)
	{
		charCount++;
		Send8Spi(str[a]);
		if(str[a]== 0)
			break;
	}
	for(int a=0;a<(4-(charCount%4));a++)
	 Send8Spi(0);
}

void TouchRegCalib (Ft_Gpu_Hal_Context_t *host)
{
  Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_A,TR1);
	Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_B,TR2);
	Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_C,TR3);
	Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_D,TR4);
	Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_E,TR5);
	Ft_Gpu_Hal_Wr32(host,REG_TOUCH_TRANSFORM_F,TR6);
}
	
	
void DmaWriteInit(uint8_t *buffer,uint32_t count)
{
	DMA0->ERQ = (uint32_t)(1<<DMA_TX_CHNL | 1<<DMA_RX_CHNL);
	
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
}

void DmaReadInit(uint8_t *buffer,uint32_t count)
{
  DMA0->ERQ = (uint32_t)(1<<DMA_TX_CHNL | 1<<DMA_RX_CHNL);
	
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
}

void ActivateFT81x(Ft_Gpu_Hal_Context_t* phost,Ft_Gpu_HalInit_t* Lcd_spiModule,Ft_Gpu_Hal_Config_t* Lcd_Spi_def_config)
{
	int check=0;
  
	while(!Ft_Gpu_Hal_Init(Lcd_spiModule))
	{}
		
	if(def_SPI_config)
		phost->hal_config = *Lcd_Spi_def_config;
	

	Ft_Gpu_Hal_Open(phost,Lcd_spiModule);
	phost->ft_cmd_fifo_wp = 0;
	phost->ft_dl_buff_wp = 0;

	Ft_Gpu_HostCommand(phost,FT_GPU_EXTERNAL_OSC);
	Ft_Gpu_HostCommand(phost,FT_GPU_ACTIVE_M);
	
	while(check != 0x7C)
	check = Ft_Gpu_Hal_Rd8(phost, REG_ID); 
	
	Ft_Gpu_Hal_Wr8(phost,REG_CPURESET,0x1);
	Ft_Gpu_Hal_Wr16(phost,REG_CMD_WRITE,0x0);
	Ft_Gpu_Hal_Wr16(phost,REG_CMD_READ,0x0);
	Ft_Gpu_Hal_Wr8(phost,REG_CPURESET,0x0);
	while(check != 0x0)
	check = Ft_Gpu_Hal_Rd16(phost,REG_CMD_READ);
  Ft_Gpu_CoCmd_ColdStart(phost);
      
  TouchRegCalib(phost);
	
	Ft_Gpu_Hal_Wr8(phost, REG_GPIO, FT_DispPIN);
	Ft_Gpu_Hal_Wr8(phost, REG_PCLK_POL, 0x1);
	Ft_Gpu_Hal_Wr8(phost, REG_PCLK, 0x5); 
	Ft_Gpu_Hal_Wr8(phost, REG_CMD_DL, 0x0); 
}
	
                            /////////////////////////////////////////////////////////////////////////////////////////////////

ft_uint32_t Ft_CmdBuffer_Index;
ft_uint32_t Ft_DlBuffer_Index;
ft_uint8_t  Ft_DlBuffer[FT_DL_SIZE];
ft_uint8_t  Ft_CmdBuffer[FT_CMD_FIFO_SIZE];

ft_void_t Ft_App_WrCoCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd)
{
#ifdef  BUFFER_OPTIMIZATION
   /* Copy the command instruction into buffer */
   ft_uint32_t *pBuffcmd;
   pBuffcmd =(ft_uint32_t*)&Ft_CmdBuffer[Ft_CmdBuffer_Index];
   *pBuffcmd = cmd;
#endif
   /* Increment the command index */
   Ft_CmdBuffer_Index += FT_CMD_SIZE;
}

ft_void_t Ft_App_WrDlCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd)
{
#ifdef BUFFER_OPTIMIZATION
   /* Copy the command instruction into buffer */
   ft_uint32_t *pBuffcmd;
   pBuffcmd =(ft_uint32_t*)&Ft_DlBuffer[Ft_DlBuffer_Index];
   *pBuffcmd = cmd;
#endif

   /* Increment the command index */
   Ft_DlBuffer_Index += FT_CMD_SIZE;
}

ft_void_t Ft_App_WrCoStr_Buffer(Ft_Gpu_Hal_Context_t *phost,const ft_char8_t *s)
{
#ifdef  BUFFER_OPTIMIZATION
  ft_uint16_t length = 0;
  length = strlen(s) + 1;//last for the null termination

  strcpy(&Ft_CmdBuffer[Ft_CmdBuffer_Index],s);

  /* increment the length and align it by 4 bytes */
  Ft_CmdBuffer_Index += ((length + 3) & ~3);
#endif
}

ft_void_t Ft_App_Flush_DL_Buffer(Ft_Gpu_Hal_Context_t *phost)
{
#ifdef  BUFFER_OPTIMIZATION
   if (Ft_DlBuffer_Index> 0)
     Ft_Gpu_Hal_WrMem(phost,RAM_DL,Ft_DlBuffer,Ft_DlBuffer_Index);
#endif
   Ft_DlBuffer_Index = 0;
}

ft_void_t Ft_App_Flush_Co_Buffer(Ft_Gpu_Hal_Context_t *phost)
{
#ifdef  BUFFER_OPTIMIZATION
   if (Ft_CmdBuffer_Index > 0)
     Ft_Gpu_Hal_WrCmdBuf(phost,Ft_CmdBuffer,Ft_CmdBuffer_Index);
#endif
   Ft_CmdBuffer_Index = 0;
}


ft_bool_t  Ft_Gpu_Hal_Init(Ft_Gpu_HalInit_t *Lcd_spiModule)
{
  
	Lcd_spiModule->spiModule = SPI_MODULE_NO;
//  initialize pins and clock
	SIM->SCGC5 |= SIM_SCGC5_PORTD(1);
	
	PORTD->PCR[11] = PORT_PCR_MUX(2);
	PORTD->PCR[12] = PORT_PCR_MUX(2);
	PORTD->PCR[13] = PORT_PCR_MUX(2);
	PORTD->PCR[14] = PORT_PCR_MUX(2);
	
//	SPI clock enable
	clock_ip_name_t const s_dspiClock[] = DSPI_CLOCKS;
	CLOCK_EnableClock(s_dspiClock[DSPI_GetInstance(Lcd_spiModule->spiModule)]);
	
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


}

   /////////////////////////////// HAL functions //////////////////////////

ft_bool_t    Ft_Gpu_Hal_Open(Ft_Gpu_Hal_Context_t *Lcd_Spi_Handler, Ft_Gpu_HalInit_t *halmodule)
{
    ft_bool_t ret = TRUE;

	halmodule->spiModule->MCR = SPI_MCR_HALT_MASK | SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(1)  | SPI_MCR_ROOE_MASK | SPI_MCR_CONT_SCKE(Lcd_Spi_Handler->hal_config.contClock);
	halmodule->spiModule->CTAR[0] = SPI_CTAR_FMSZ(7);
  halmodule->spiModule->MCR &= ~(SPI_MCR_HALT(1)); 

    return ret;	
}

ft_void_t  Ft_Gpu_Hal_Close(Ft_Gpu_Hal_Context_t *host, Ft_Gpu_HalInit_t *halmodule)
{	
  halmodule->spiModule->MCR = SPI_MCR_HALT_MASK;
}

ft_void_t Ft_Gpu_Hal_DeInit()
{

  //  de initialize pins and clock
	SIM->SCGC5 |= SIM_SCGC5_PORTD(0);
	
	PORTD->PCR[11] = PORT_PCR_MUX(2);
	PORTD->PCR[12] = PORT_PCR_MUX(2);
	PORTD->PCR[13] = PORT_PCR_MUX(2);
	PORTD->PCR[14] = PORT_PCR_MUX(2);
	
	SIM->SCGC3 = SIM_SCGC3_SPI2(0);
}


ft_void_t  Ft_Gpu_Hal_StartTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw,ft_uint32_t addr)
{
	
	
	
	if (FT_GPU_READ == rw)
	{ 
		ReadFrmAdd (addr);

		 Send8Dummy(); //Dummy Read Byte

		host->status = FT_GPU_HAL_READING;
	}
	else
	{
	 WriteToAdd (addr);
   
		host->status = FT_GPU_HAL_WRITING;
	}
}


ft_void_t  Ft_Gpu_Hal_StartCmdTransfer(Ft_Gpu_Hal_Context_t *host,FT_GPU_TRANSFERDIR_T rw, ft_uint16_t count)
{     
    Ft_Gpu_Hal_StartTransfer(host,rw,host->ft_cmd_fifo_wp + RAM_CMD);    
}



ft_uint8_t    Ft_Gpu_Hal_TransferString(Ft_Gpu_Hal_Context_t *host,const ft_char8_t *string)
{    
    ft_uint16_t length = strlen(string);
    while(length --)
    {
        Ft_Gpu_Hal_Transfer8(host,*string);
        string ++;
    }
    //Append one null as ending flag
    Ft_Gpu_Hal_Transfer8(host,0);    
}

ft_uint8_t    Ft_Gpu_Hal_Transfer8(Ft_Gpu_Hal_Context_t *host,ft_uint8_t value)
{ 
      ft_uint8_t ReadByte = 0;
	    if (host->status == FT_GPU_HAL_WRITING)
	    {
		    Send8Spi(value);
	    }
	    else
	    {
		    ReadByte = Read8 ();
	    }
	    return ReadByte;
}


ft_uint16_t  Ft_Gpu_Hal_Transfer16(Ft_Gpu_Hal_Context_t *host,ft_uint16_t value)
{
	ft_uint16_t retVal = 0;

    if (host->status == FT_GPU_HAL_WRITING)
	{
		Ft_Gpu_Hal_Transfer8(host,(uint8_t)value);//LSB first
		Ft_Gpu_Hal_Transfer8(host,(uint8_t)(value >> 8));
	}
	else
	{
		retVal = Ft_Gpu_Hal_Transfer8(host,0);
		retVal |= (ft_uint16_t)Ft_Gpu_Hal_Transfer8(host,0) << 8;
	}

	return retVal;
}


ft_uint32_t  Ft_Gpu_Hal_Transfer32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t value)
{
	ft_uint32_t retVal = 0;
    
	if (host->status == FT_GPU_HAL_WRITING)
	{
		Ft_Gpu_Hal_Transfer16(host,(uint16_t)value );//LSB first
		Ft_Gpu_Hal_Transfer16(host,(value >> 16));
	}
	else
	{
		retVal = Ft_Gpu_Hal_Transfer16(host,0);
		retVal |= (ft_uint32_t)Ft_Gpu_Hal_Transfer16(host,0) << 16;
	}
	return retVal;
}


ft_void_t   Ft_Gpu_Hal_EndTransfer(Ft_Gpu_Hal_Context_t *host)
{

	CeaseTrans();

	host->status = FT_GPU_HAL_OPENED;
}


ft_uint8_t  Ft_Gpu_Hal_Rd8(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr)
{
	ft_uint8_t value = 0;  
	
			Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
			value = Ft_Gpu_Hal_Transfer8(host,0);
            Ft_Gpu_Hal_EndTransfer(host);

	return value;
}


ft_uint16_t Ft_Gpu_Hal_Rd16(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr)
{
		ft_uint16_t value;

		Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
		value = Ft_Gpu_Hal_Transfer16(host, 0);
    Ft_Gpu_Hal_EndTransfer(host);

		
	return value;
}

ft_uint32_t Ft_Gpu_Hal_Rd32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr)
{
	  ft_uint32_t value;
		Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
		value = Ft_Gpu_Hal_Transfer32(host,0);
    Ft_Gpu_Hal_EndTransfer(host);
	
	return value;
}


ft_void_t Ft_Gpu_Hal_Wr8(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint8_t v)
{	

		Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
		Ft_Gpu_Hal_Transfer8(host,v);
		Ft_Gpu_Hal_EndTransfer(host);

}

ft_void_t Ft_Gpu_Hal_Wr16(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint16_t v)
{

		Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
		Ft_Gpu_Hal_Transfer16(host,v);
		Ft_Gpu_Hal_EndTransfer(host);

}


ft_void_t Ft_Gpu_Hal_Wr32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint32_t v)
{

		Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
		Ft_Gpu_Hal_Transfer32(host,v);
		Ft_Gpu_Hal_EndTransfer(host);

}


ft_void_t Ft_Gpu_HostCommand(Ft_Gpu_Hal_Context_t *host,ft_uint8_t cmd)
{
	uint8_t cmdArr[3]={cmd,0,0};
  for(int a=0; a<3;a++)
	{
	 Send8Spi(cmdArr[a]);
  }
	CeaseTrans();
}



ft_void_t Ft_Gpu_ClockSelect(Ft_Gpu_Hal_Context_t *host,FT_GPU_PLL_SOURCE_T pllsource)
{
   Ft_Gpu_HostCommand(host,pllsource);
}


ft_void_t Ft_Gpu_PLL_FreqSelect(Ft_Gpu_Hal_Context_t *host,FT_GPU_PLL_FREQ_T freq)
{
   Ft_Gpu_HostCommand(host,freq);
}


ft_void_t Ft_Gpu_PowerModeSwitch(Ft_Gpu_Hal_Context_t *host,FT_GPU_POWER_MODE_T pwrmode)
{
   Ft_Gpu_HostCommand(host,pwrmode);
}



ft_void_t Ft_Gpu_CoreReset(Ft_Gpu_Hal_Context_t *host)
{
   Ft_Gpu_HostCommand(host,FT_GPU_CORE_RESET);
}


#ifdef FT_81X_ENABLE
ft_void_t Ft_Gpu_81X_SelectSysCLK(Ft_Gpu_Hal_Context_t *host, FT_GPU_81X_PLL_FREQ_T freq)
{
		if(FT_GPU_SYSCLK_72M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (ft_uint32_t)0x61 | (0x40 << 8) | (0x06 << 8)); 
		else if(FT_GPU_SYSCLK_60M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (ft_uint32_t)0x61 | (0x40 << 8) | (0x05 << 8)); 
		else if(FT_GPU_SYSCLK_48M == freq)    
			Ft_Gpu_HostCommand_Ext3(host, (ft_uint32_t)0x61 | (0x40 << 8) | (0x04 << 8)); 
		else if(FT_GPU_SYSCLK_36M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (ft_uint32_t)0x61 | (0x03 << 8)); 
		else if(FT_GPU_SYSCLK_24M == freq)
			Ft_Gpu_HostCommand_Ext3(host, (ft_uint32_t)0x61 | (0x02 << 8)); 
		else if(FT_GPU_SYSCLK_DEFAULT == freq)//default clock
			Ft_Gpu_HostCommand_Ext3(host, 0x61); 
}

ft_void_t Ft_GPU_81X_PadDriveStrength(Ft_Gpu_Hal_Context_t *host, FT_GPU_81X_GPIO_DRIVE_STRENGTH_T strength, FT_GPU_81X_GPIO_GROUP_T group)
{
		Ft_Gpu_HostCommand_Ext3(host, (ft_uint32_t)0x70 | (group << 8) | (strength << 8));
}


ft_void_t Ft_Gpu_81X_ResetActive(Ft_Gpu_Hal_Context_t *host)
{
	Ft_Gpu_HostCommand_Ext3(host, FT_GPU_81X_RESET_ACTIVE); 
}


ft_void_t Ft_Gpu_81X_ResetRemoval(Ft_Gpu_Hal_Context_t *host)
{
	Ft_Gpu_HostCommand_Ext3(host, FT_GPU_81X_RESET_REMOVAL); 
}
#endif



ft_void_t Ft_Gpu_HostCommand_Ext3(Ft_Gpu_Hal_Context_t *host,ft_uint32_t cmd)
{

	 uint8_t cmdArr[3] = {0};
	 cmdArr[0] = (uint8_t)cmd;
	 cmdArr[1] = (uint8_t)(cmd >> 8); 
	 cmdArr[2] = (uint8_t)(cmd >> 16); 
   
	 for(int a=0; a<3;a++)
	{
	 Send8Spi(cmdArr[a]);
  }
	CeaseTrans();
	

}


ft_void_t Ft_Gpu_Hal_Updatecmdfifo(Ft_Gpu_Hal_Context_t *host,ft_uint32_t count)
{
	host->ft_cmd_fifo_wp  = (host->ft_cmd_fifo_wp + count) & 4095;

	//4 byte alignment
	host->ft_cmd_fifo_wp = (host->ft_cmd_fifo_wp + 3) & 0xffc;
	Ft_Gpu_Hal_Wr16(host,REG_CMD_WRITE,host->ft_cmd_fifo_wp); 

}


ft_uint16_t Ft_Gpu_Cmdfifo_Freespace(Ft_Gpu_Hal_Context_t *host)
{
	ft_uint16_t fullness,retval;

	//host->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);

	fullness = (host->ft_cmd_fifo_wp - Ft_Gpu_Hal_Rd16(host,REG_CMD_READ)) & 4095;
	retval = (FT_CMD_FIFO_SIZE - 4) - fullness;
	return (retval);
}




ft_void_t Ft_Gpu_Hal_WrCmdBuf(Ft_Gpu_Hal_Context_t *host,ft_uint8_t *buffer,ft_uint32_t count)
{
	ft_uint32_t length =0, SizeTransfered = 0,availablefreesize;

	do 
	{                
		length = count;
        availablefreesize = Ft_Gpu_Cmdfifo_Freespace(host);

		if (length > availablefreesize)
		{
		    length = availablefreesize;
		}  
      Ft_Gpu_Hal_CheckCmdBuffer(host,length);
   
		  Ft_Gpu_Hal_StartCmdTransfer(host,FT_GPU_WRITE,length); 
	
		SizeTransfered = 0;
		DmaWriteInit(buffer,length);
	  ClearFlags();
		SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
		length = count;
		while(!dmaDone)
		{}
			dmaDone = 0;
		SPI_MODULE_NO->RSER = 0;
		
				
//			    while (length--) {
//			    Ft_Gpu_Hal_Transfer8(host,*buffer);
//			    buffer++;
//			    SizeTransfered ++;
//			    }
//			    length = SizeTransfered;

		    
		Ft_Gpu_Hal_EndTransfer(host);			
		Ft_Gpu_Hal_Updatecmdfifo(host,length);
		Ft_Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while(count > 0);
}


/***************************************************************************
* Interface Description    : Blocking function call
*							 Blocks until "count" number of bytes gets available
*							 in RAM_CMD
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_CheckCmdBuffer(Ft_Gpu_Hal_Context_t *host,ft_uint32_t count)
{
   ft_uint16_t getfreespace;
   do{
        getfreespace = Ft_Gpu_Cmdfifo_Freespace(host);
   }while(getfreespace < count);
}

/***************************************************************************
* Interface Description    : Blocking function call
*							 Blocks until all commands in RAM_CMD are executed and 
*							 it is fully empty
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_WaitCmdfifo_empty(Ft_Gpu_Hal_Context_t *host)
{
   while(Ft_Gpu_Hal_Rd16(host,REG_CMD_READ) != Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE));
//   host->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
}

/***************************************************************************
* Interface Description    : Continuous write to RAM_CMD with no wait
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_WrCmdBuf_nowait(Ft_Gpu_Hal_Context_t *host,ft_uint8_t *buffer,ft_uint32_t count)
{
	ft_uint32_t length =0, SizeTransfered = 0 , availablefreesize;
	

	do {                
			length = count;
            availablefreesize = Ft_Gpu_Cmdfifo_Freespace(host);

			if (length > availablefreesize)
			{
				length = availablefreesize;
			}
      	    Ft_Gpu_Hal_CheckCmdBuffer(host,length);
            Ft_Gpu_Hal_StartCmdTransfer(host,FT_GPU_WRITE,length);
		
				SizeTransfered = 0;
			
		DmaWriteInit(buffer,length);
	  ClearFlags();
		SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
		length = count;
		while(!dmaDone)
		{}
			dmaDone = 0;
		SPI_MODULE_NO->RSER = 0;
			
//				while (length--)
//				{
//                    Ft_Gpu_Hal_Transfer8(host,*buffer);
//		            buffer++;
//                    SizeTransfered ++;
//		        }
//                length = SizeTransfered;
       
   

			Ft_Gpu_Hal_EndTransfer(host);
			Ft_Gpu_Hal_Updatecmdfifo(host,length);

	//	Ft_Gpu_Hal_WaitCmdfifo_empty(host);

		count -= length;
	}while (count > 0);
}

/***************************************************************************
* Interface Description    : 
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_uint8_t Ft_Gpu_Hal_WaitCmdfifo_empty_status(Ft_Gpu_Hal_Context_t *host)
{
   if(Ft_Gpu_Hal_Rd16(host,REG_CMD_READ) != Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE))
   {
     return 0;
   }
   else
   {
     host->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
     return 1;
   }  
}

/***************************************************************************
* Interface Description    :
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_WaitLogo_Finish(Ft_Gpu_Hal_Context_t *host)
{
    ft_int16_t cmdrdptr,cmdwrptr;

    do{
         cmdrdptr = Ft_Gpu_Hal_Rd16(host,REG_CMD_READ);
         cmdwrptr = Ft_Gpu_Hal_Rd16(host,REG_CMD_WRITE);
    }while ((cmdwrptr != cmdrdptr) || (cmdrdptr != 0));
    host->ft_cmd_fifo_wp = 0;
}

/***************************************************************************
* Interface Description    :
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_ResetCmdFifo(Ft_Gpu_Hal_Context_t *host)
{
   host->ft_cmd_fifo_wp = 0;
}

/***************************************************************************
* Interface Description    :
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_WrCmd32(Ft_Gpu_Hal_Context_t *host,ft_uint32_t cmd)
{
         Ft_Gpu_Hal_CheckCmdBuffer(host,sizeof(cmd));
      
         Ft_Gpu_Hal_Wr32(host,RAM_CMD + host->ft_cmd_fifo_wp,cmd);
      
         Ft_Gpu_Hal_Updatecmdfifo(host,sizeof(cmd));
}

/***************************************************************************
* Interface Description    :
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_ResetDLBuffer(Ft_Gpu_Hal_Context_t *host)
{
           host->ft_dl_buff_wp = 0;
}


/***************************************************************************
* Interface Description    : Toggle PD_N pin of FT800 board for a power cycle
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
//ft_void_t Ft_Gpu_Hal_Powercycle(Ft_Gpu_Hal_Context_t *host, ft_bool_t up)
//{

//	if (up)
//	{
//		
//		#ifdef ARDUINO_PLATFORM      
//			digitalWrite(host->hal_config.pdnPinNo, LOW);
//			Ft_Gpu_Hal_Sleep(20);

//			digitalWrite(host->hal_config.pdnPinNo, HIGH);
//			Ft_Gpu_Hal_Sleep(20);
//		#endif
//		
//		#ifdef FT900_PLATFORM
//			gpio_write(host->hal_config.pdnPinNo, 0);
//			ft_delay(20);
//			gpio_write(host->hal_config.pdnPinNo, 1);
//			ft_delay(20);
//		#endif
//	}
//	
//	else
//	{
//		
//		#ifdef ARDUINO_PLATFORM
//			digitalWrite(host->hal_config.pdnPinNo, HIGH);
//			Ft_Gpu_Hal_Sleep(20);
//            
//            digitalWrite(host->hal_config.pdnPinNo, LOW);
//            Ft_Gpu_Hal_Sleep(20);
//#endif
//#ifdef FT900_PLATFORM
//            gpio_write(host->hal_config.pdnPinNo, 1);
//            ft_delay(20);
//            gpio_write(host->hal_config.pdnPinNo, 0);
//            ft_delay(20);
//#endif

//	}
//}

/***************************************************************************
* Interface Description    : Ft_Gpu_Hal_WrMemFromFlash and Ft_Gpu_Hal_WrMem ideally
*                            perform same operation.Find why was 2 created?
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_WrMemFromFlash(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint8_t *buffer, ft_uint32_t length)
{
	ft_uint32_t SizeTransfered = 0;      

    Ft_Gpu_Hal_StartTransfer(host, FT_GPU_WRITE, addr);
	  
			
	  if(length<=DMA_MAX_ITER)
	  {
					DmaWriteInit(buffer,length);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
		}
		else 
    {
		  do{
					DmaWriteInit(buffer,DMA_MAX_ITER);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
					buffer+=DMA_MAX_ITER;
					length-=DMA_MAX_ITER;
				}while(length>DMA_MAX_ITER);
			    DmaWriteInit(buffer,length);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
		}
			
//	while (length--) {
//            Ft_Gpu_Hal_Transfer8(host,ft_pgm_read_byte_near(buffer));
//	    buffer++;
//	}
    Ft_Gpu_Hal_EndTransfer(host);

}

/***************************************************************************
* Interface Description    : 
*                            
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_WrMem(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uchar8_t *buffer, ft_uint32_t length)
{
	  ft_uint32_t SizeTransfered = 0;      
		Ft_Gpu_Hal_StartTransfer(host,FT_GPU_WRITE,addr);
  
	  if(length<=DMA_MAX_ITER)
	  {
					DmaWriteInit(buffer,length);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
		}
		else 
    {
		  do{
					DmaWriteInit(buffer,DMA_MAX_ITER);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
					buffer+=DMA_MAX_ITER;
					length-=DMA_MAX_ITER;
				}while(length>DMA_MAX_ITER);
			    DmaWriteInit(buffer,length);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
		}
			
	
//			while (length--)
//			{
//				Ft_Gpu_Hal_Transfer8(host,*buffer);
//				buffer++;
//			}
//        Ft_Gpu_Hal_EndTransfer(host);

}

/***************************************************************************
* Interface Description    : 
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Gpu_Hal_RdMem(Ft_Gpu_Hal_Context_t *host,ft_uint32_t addr, ft_uint8_t *buffer, ft_uint32_t length)
{
  	ft_uint32_t SizeTransfered = 0;      
		Ft_Gpu_Hal_StartTransfer(host,FT_GPU_READ,addr);
	
	  if(length<=DMA_MAX_ITER)
	  {
					DmaReadInit(buffer,length);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
		}
		else
		{
		  do{
					DmaReadInit(buffer,DMA_MAX_ITER);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
					buffer+=DMA_MAX_ITER;
					length-=DMA_MAX_ITER;
				}while(length>DMA_MAX_ITER);
			    DmaReadInit(buffer,length);
					ClearFlags();
					SPI_MODULE_NO->RSER = SPI_RSER_RFDF_DIRS_MASK | SPI_RSER_RFDF_RE_MASK |SPI_RSER_TFFF_DIRS(1) | SPI_RSER_TFFF_RE(1);
					while(!dmaDone)
					{}
						dmaDone = 0;
					SPI_MODULE_NO->RSER = 0;
		}			
//			while (length--) {
//			   *buffer = Ft_Gpu_Hal_Transfer8(host,0);
//			   buffer++;
//			}

        Ft_Gpu_Hal_EndTransfer(host);
	
}

/***************************************************************************
* Interface Description    : Helper api for dec to ascii
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_int32_t Ft_Gpu_Hal_Dec2Ascii(ft_char8_t *pSrc,ft_int32_t value)
{
	ft_int16_t Length;
	ft_char8_t *pdst,charval;
	ft_int32_t CurrVal = value,tmpval,i;
	ft_char8_t tmparray[16],idx = 0;

	Length = strlen(pSrc);
	pdst = pSrc + Length;

	if(0 == value)
	{
		*pdst++ = '0';
		*pdst++ = '\0';
		return 0;
	}

	if(CurrVal < 0)
	{
		*pdst++ = '-';
		CurrVal = - CurrVal;
	}
	/* insert the value */
	while(CurrVal > 0){
		tmpval = CurrVal;
		CurrVal /= 10;
		tmpval = tmpval - CurrVal*10;
		charval = '0' + tmpval;
		tmparray[idx++] = charval;
	}

	for(i=0;i<idx;i++)
	{
		*pdst++ = tmparray[idx - i - 1];
	}
	*pdst++ = '\0';

	return 0;
}

/***************************************************************************
* Interface Description    : Calls platform specific sleep call
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
//ft_void_t Ft_Gpu_Hal_Sleep(ft_uint32_t ms)
//{
//#ifdef FT900_PLATFORM
//	delayms(ms);
//#endif
//#if defined(MSVC_PLATFORM) || defined(MSVC_FT800EMU)
//	Sleep(ms);
//#endif
//#ifdef ARDUINO_PLATFORM
//	delay(ms);
//#endif
//}

/***************************************************************************
* Interface Description    : Set EVE spi communication mode
*                            Set USB bridge communication mode
*                            Update global variable
* Implementation           :
* Return Value             : ft_void_t
*                            -1 - Error, 0 - Success
* Author                   :
****************************************************************************/
#ifdef FT_81X_ENABLE
ft_int16_t Ft_Gpu_Hal_SetSPI(Ft_Gpu_Hal_Context_t *host,FT_GPU_SPI_NUMCHANNELS_T numchnls,FT_GPU_SPI_NUMDUMMYBYTES numdummy)
{
	ft_uint8_t writebyte = 0;
	
 //function not necessary
 
	return 0;
}
#endif




/***************************************************************************
* Interface Description    : FIFO related apis
*                            Init all the parameters of fifo buffer
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Fifo_Init(Ft_Fifo_t *pFifo,ft_uint32_t StartAddress,ft_uint32_t Length,ft_uint32_t HWReadRegAddress,ft_uint32_t HWWriteRegAddress)
{
	/* update the context parameters */
	pFifo->fifo_buff = StartAddress;
	pFifo->fifo_len = Length;
	pFifo->fifo_rp = pFifo->fifo_wp = 0;

	/* update the hardware register addresses - specific to FT800 series chips */
	pFifo->HW_Read_Reg = HWReadRegAddress;
	pFifo->HW_Write_Reg = HWWriteRegAddress;
}

/***************************************************************************
* Interface Description    : FIFO related apis
*                            update both the read and write pointers
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Fifo_Update(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo)
{
	pFifo->fifo_rp = Ft_Gpu_Hal_Rd32(host,pFifo->HW_Read_Reg);
	//Ft_Gpu_Hal_Wr32(host,pFifo->HW_Write_Reg,pFifo->fifo_wp);
}

/***************************************************************************
* Interface Description    : FIFO related apis
*                            just write and update the write register
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_uint32_t Ft_Fifo_Write(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo,ft_uint8_t *buffer,ft_uint32_t NumbytetoWrite)
{
	ft_uint32_t FreeSpace = Ft_Fifo_GetFreeSpace(host,pFifo),TotalBytes = NumbytetoWrite;

	if(NumbytetoWrite > FreeSpace)
	{
		/* update the read pointer and get the free space */
		Ft_Fifo_Update(host,pFifo);
		FreeSpace = Ft_Fifo_GetFreeSpace(host,pFifo);

		if(NumbytetoWrite > FreeSpace)
		{
			TotalBytes = FreeSpace;
		}
	}

	/* sanity check */
	if(TotalBytes <= 0)
	{
		//printf("no space in fifo write %d %d %d %d\n",TotalBytes,FreeSpace,pFifo->fifo_wp,pFifo->fifo_rp);
		return 0;//error condition
	}
	/* check for the loopback conditions */
	if(pFifo->fifo_wp + TotalBytes >= pFifo->fifo_len)
	{
		ft_uint32_t partialchunk = pFifo->fifo_len - pFifo->fifo_wp,secpartialchunk = TotalBytes - partialchunk;

		Ft_Gpu_Hal_WrMem(host,pFifo->fifo_buff + pFifo->fifo_wp,buffer,partialchunk);
		if(secpartialchunk > 0)
		{
			Ft_Gpu_Hal_WrMem(host,pFifo->fifo_buff,buffer + partialchunk,secpartialchunk);
		}
		pFifo->fifo_wp = secpartialchunk;
		//printf("partial chunks %d %d %d %d\n",partialchunk,secpartialchunk,pFifo->fifo_wp,pFifo->fifo_rp);

	}
	else
	{
		Ft_Gpu_Hal_WrMem(host,pFifo->fifo_buff + pFifo->fifo_wp,buffer,TotalBytes);
		pFifo->fifo_wp += TotalBytes;
	}

	/* update the write pointer address in write register */
	Ft_Gpu_Hal_Wr32(host,pFifo->HW_Write_Reg,pFifo->fifo_wp);

	return TotalBytes;
}

/***************************************************************************
* Interface Description    : FIFO related apis
*                            just write one word and update the write register
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Fifo_Write32(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo,ft_uint32_t WriteWord)
{
	Ft_Fifo_Write(host,pFifo,(ft_uint8_t *)&WriteWord,4);
}

/***************************************************************************
* Interface Description    : FIFO related apis
*                            write and wait for the fifo to be empty. handle cases even if
*                            the Numbytes are more than freespace
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_void_t Ft_Fifo_WriteWait(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo,ft_uint8_t *buffer,ft_uint32_t Numbyte)
{
	ft_uint32_t TotalBytes = Numbyte,currchunk = 0,FreeSpace;
	ft_uint8_t *pbuff = buffer;
	/* blocking call, manage to check for the error case and break in case of error */
	while(TotalBytes > 0)
	{
		currchunk = TotalBytes;
		FreeSpace = Ft_Fifo_GetFreeSpace(host,pFifo);
		if(currchunk > FreeSpace)
		{
			currchunk = FreeSpace;
		}

		Ft_Fifo_Write(host,pFifo,pbuff,currchunk);
		pbuff += currchunk;
		TotalBytes -= currchunk;


	}
}



/***************************************************************************
* Interface Description    : FIFO related apis
*                            get the free space in the fifo - make sure the 
*                            return value is maximum of (LENGTH - 4)
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
ft_uint32_t Ft_Fifo_GetFreeSpace(Ft_Gpu_Hal_Context_t *host,Ft_Fifo_t *pFifo)
{
	ft_uint32_t FreeSpace = 0;

	Ft_Fifo_Update(host,pFifo);

	if(pFifo->fifo_wp >= pFifo->fifo_rp)
	{
		FreeSpace = pFifo->fifo_len - pFifo->fifo_wp + pFifo->fifo_rp;
	}
	else
	{
		FreeSpace = pFifo->fifo_rp - pFifo->fifo_wp;
	}

	if(FreeSpace >= 4)
	{
		FreeSpace -= 4;//make sure 1 word space is maintained between rd and wr pointers
	}
	return FreeSpace;
}

/***************************************************************************
* Interface Description    : 
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
//ft_uint32_t Ft_Gpu_CurrentFrequency(Ft_Gpu_Hal_Context_t *host)
//{
//    ft_uint32_t t0, t1;
//    ft_uint32_t addr = REG_CLOCK;
//    ft_uint8_t spidata[4];
//	ft_int32_t r = 15625;

//    t0 = Ft_Gpu_Hal_Rd32(host,REG_CLOCK); /* t0 read */
//               
//   //delay of 15625 micorseconds

//    t1 = Ft_Gpu_Hal_Rd32(host,REG_CLOCK); /* t1 read */
//    return ((t1 - t0) * 64); /* bitshift 6 places is the same as multiplying 64 */
//}

/***************************************************************************
* Interface Description    :
* Implementation           :
* Return Value             : ft_void_t
* Author                   :
****************************************************************************/
//ft_int32_t Ft_Gpu_ClockTrimming(Ft_Gpu_Hal_Context_t *host,ft_int32_t LowFreq)
//{
//   ft_uint32_t f;
//   ft_uint8_t i;

//  /* Trim the internal clock by increase the REG_TRIM register till the measured frequency is within the acceptable range.*/
//   for (i=0; (i < 31) && ((f= Ft_Gpu_CurrentFrequency(host)) < LowFreq); i++)
//   {
//	   Ft_Gpu_Hal_Wr8(host,REG_TRIM, i);  /* increase the REG_TRIM register value automatically increases the internal clock */

//   }

//   Ft_Gpu_Hal_Wr32(host,REG_FREQUENCY,f);  /* Set the final frequency to be used for internal operations */

//   return f;
//}
 
 
                                                         ////////////////////////// image /////////////////////
																												 
//	void add_image(uint8_t* ptoimg,ft_uint32_t size)
//{
//  ft_uint8_t *pBuffcmd;
//   pBuffcmd =(ft_uint8_t*)&Ft_CmdBuffer[Ft_CmdBuffer_Index];
//	for(int a=0;a<=size;a++)
//  {
//		*pBuffcmd = *ptoimg;
//		ptoimg++;
//	}
//	Ft_CmdBuffer_Index += size;
//	Ft_CmdBuffer_Index = (Ft_CmdBuffer_Index + 3) & 0xffc;
//}				

/* EOF */





																			


					 																							
													 
