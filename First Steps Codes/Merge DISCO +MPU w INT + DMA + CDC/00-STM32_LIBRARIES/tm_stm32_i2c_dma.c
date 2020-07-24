/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen MAJERLE
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_i2c_dma.h"
#include "tm_stm32_i2c.h"


/* Private structure */
typedef struct {
	uint32_t TX_Channel;
	DMA_Stream_TypeDef* TX_Stream;
	uint32_t RX_Channel;
	DMA_Stream_TypeDef* RX_Stream;
	uint32_t Dummy32;
	uint16_t Dummy16;
	I2C_HandleTypeDef Handle;
} TM_I2C_DMA_INT_t;

/* Private variables */
#ifdef I2C1
static TM_I2C_DMA_INT_t I2C1_DMA_INT = {I2C1_DMA_TX_CHANNEL, I2C1_DMA_TX_STREAM, I2C1_DMA_RX_CHANNEL, I2C1_DMA_RX_STREAM};
#endif
#ifdef I2C2
static TM_I2C_DMA_INT_t I2C2_DMA_INT = {I2C2_DMA_TX_CHANNEL, I2C2_DMA_TX_STREAM, I2C2_DMA_RX_CHANNEL, I2C2_DMA_RX_STREAM};
#endif
#ifdef I2C3
static TM_I2C_DMA_INT_t I2C3_DMA_INT = {I2C3_DMA_TX_CHANNEL, I2C3_DMA_TX_STREAM, I2C3_DMA_RX_CHANNEL, I2C3_DMA_RX_STREAM};
#endif

/* Private functions */
static TM_I2C_DMA_INT_t* TM_I2C_DMA_INT_GetSettings(I2C_TypeDef* I2Cx);
	
void TM_I2C_DMA_Init(I2C_TypeDef* I2Cx) {
	/* Init DMA TX mode */
	/* Assuming I2C is already initialized and clock is enabled */
	
	/* Get USART settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Init both streams */
	TM_DMA_Init(Settings->TX_Stream, NULL);
	TM_DMA_Init(Settings->RX_Stream, NULL);
}

void TM_I2C_DMA_InitWithStreamAndChannel(I2C_TypeDef* I2Cx, DMA_Stream_TypeDef* TX_Stream, uint32_t TX_Channel, DMA_Stream_TypeDef* RX_Stream, uint32_t RX_Channel) {
	/* Get USART settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Set values */
	Settings->RX_Channel = RX_Channel;
	Settings->RX_Stream = RX_Stream;
	Settings->TX_Channel = TX_Channel;
	Settings->TX_Stream = TX_Stream;
	
	/* Init I2C */
	TM_I2C_DMA_Init(I2Cx);
}

void TM_I2C_DMA_Deinit(I2C_TypeDef* I2Cx) {
	/* Get USART settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Deinit DMA Streams */
	TM_DMA_DeInit(Settings->TX_Stream);
	TM_DMA_DeInit(Settings->RX_Stream);
}

uint8_t TM_I2C_DMA_Transmit(I2C_TypeDef* I2Cx, uint8_t* TX_Buffer, uint8_t* RX_Buffer, uint16_t count) {
	DMA_HandleTypeDef DMA_InitStruct;
	
	/* Get USART settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Check if DMA available */
	if (
		Settings->RX_Stream->NDTR || 
		Settings->TX_Stream->NDTR || 
		(TX_Buffer == NULL && RX_Buffer == NULL)
	) {
		return 0;
	}
	
#if defined(STM32F7xx)
    CLEAR_BIT(I2Cx->CR2, I2C_CR2_LDMATX);
    CLEAR_BIT(I2Cx->CR2, I2C_CR2_LDMARX);
#endif
	
	/* Set DMA default */
	DMA_InitStruct.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA_InitStruct.Init.Mode = DMA_CIRCULAR;
	DMA_InitStruct.Init.Priority = DMA_PRIORITY_LOW;
	DMA_InitStruct.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	DMA_InitStruct.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	DMA_InitStruct.Init.MemBurst = DMA_MBURST_SINGLE;
	DMA_InitStruct.Init.PeriphBurst = DMA_PBURST_SINGLE;

	/* Set dummy memory to default */
	Settings->Dummy16 = 0x12;
	
	/* Set memory size */
	DMA_InitStruct.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA_InitStruct.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;

	/*******************************************************/
	/*                       RX DMA                        */
	/*******************************************************/
	/* Set stream */
	DMA_InitStruct.Instance = Settings->RX_Stream;
	
	/* Configure RX DMA */
	DMA_InitStruct.Init.Channel = Settings->RX_Channel;
	DMA_InitStruct.Init.Direction = DMA_PERIPH_TO_MEMORY;
	DMA_InitStruct.Init.Priority = DMA_PRIORITY_HIGH;
	
	/* Deinit first RX stream */
	TM_DMA_ClearFlag(Settings->RX_Stream, DMA_FLAG_ALL);
	
	/* Set memory increase */
	if (RX_Buffer != NULL) {
		DMA_InitStruct.Init.MemInc = DMA_MINC_ENABLE;
	} else {
		DMA_InitStruct.Init.MemInc = DMA_MINC_DISABLE;
	}
	
	/* Start TX stream */
	TM_DMA_Init(Settings->RX_Stream, &DMA_InitStruct);
	
	/* Start DMA */
	if (RX_Buffer != NULL) {
		TM_DMA_Start(&DMA_InitStruct, (uint32_t) &I2Cx->DR, (uint32_t) RX_Buffer, count);
	} else {
		TM_DMA_Start(&DMA_InitStruct, (uint32_t) &I2Cx->DR, (uint32_t) &Settings->Dummy32, count);
	}
	
	/*******************************************************/
	/*                       TX DMA                        */
	/*******************************************************/
	/* Set stream */
	DMA_InitStruct.Instance = Settings->TX_Stream;
	
	/* Configure TX DMA */
	DMA_InitStruct.Init.Channel = Settings->TX_Channel;
	DMA_InitStruct.Init.Direction = DMA_MEMORY_TO_PERIPH;
	
	/* Deinit first TX stream */
	TM_DMA_ClearFlag(Settings->TX_Stream, DMA_FLAG_ALL);
	
	/* Set memory increase */
	if (TX_Buffer != NULL) {
		DMA_InitStruct.Init.MemInc = DMA_MINC_ENABLE;
	} else {
		DMA_InitStruct.Init.MemInc = DMA_MINC_DISABLE;
	}
	
	/* Start TX stream */
	TM_DMA_Init(Settings->TX_Stream, &DMA_InitStruct);
	
	/* Start DMA */
	if (TX_Buffer != NULL) {
		TM_DMA_Start(&DMA_InitStruct, (uint32_t) TX_Buffer, (uint32_t) &I2Cx->DR, count);
	} else {
		TM_DMA_Start(&DMA_InitStruct, (uint32_t) &Settings->Dummy32, (uint32_t) &I2Cx->DR, count);
	}
	
	/* Start stream */
	I2Cx->CR2 |= I2C_CR2_DMAEN;
	
	/* Return OK */
	return 1;
}

uint8_t TM_I2C_DMA_SendByte(I2C_TypeDef* I2Cx, uint8_t value, uint16_t count) {
	DMA_HandleTypeDef DMA_InitStruct;
	
	/* Get USART settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Check if DMA available */
	if (Settings->TX_Stream->NDTR) {
		return 0;
	}
	
	/* Set DMA default */
	DMA_InitStruct.Instance = Settings->TX_Stream;
	DMA_InitStruct.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMA_InitStruct.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	DMA_InitStruct.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA_InitStruct.Init.Mode = DMA_NORMAL;
	DMA_InitStruct.Init.Priority = DMA_PRIORITY_LOW;
	DMA_InitStruct.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	DMA_InitStruct.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	DMA_InitStruct.Init.MemBurst = DMA_MBURST_SINGLE;
	DMA_InitStruct.Init.PeriphBurst = DMA_PBURST_SINGLE;
	DMA_InitStruct.Init.MemInc = DMA_MINC_DISABLE;
	DMA_InitStruct.Init.Channel = Settings->TX_Channel;
	DMA_InitStruct.Init.Direction = DMA_MEMORY_TO_PERIPH;
	
	/* Set dummy memory to value we specify */
	Settings->Dummy32 = value;
	
	/* Deinit first TX stream */
	TM_DMA_ClearFlag(Settings->TX_Stream, DMA_FLAG_ALL);
	
	/* Init TX stream */
	TM_DMA_Init(Settings->TX_Stream, &DMA_InitStruct);
	
	/* Start TX stream */
	TM_DMA_Start(&DMA_InitStruct, (uint32_t) &Settings->Dummy32, (uint32_t) &I2Cx->DR, count);
	
	/* Enable I2C TX DMA */
	I2Cx->CR2 |= I2C_CR2_DMAEN;
	
	/* Return OK */
	return 1;
}

uint8_t TM_I2C_DMA_SendHalfWord(I2C_TypeDef* I2Cx, uint16_t value, uint16_t count) {
	DMA_HandleTypeDef DMA_InitStruct;
	
	/* Get USART settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Check if DMA available */
	if (Settings->TX_Stream->NDTR) {
		return 0;
	}
	
	/* Set DMA default */
	DMA_InitStruct.Instance = Settings->TX_Stream;
	DMA_InitStruct.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	DMA_InitStruct.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	DMA_InitStruct.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA_InitStruct.Init.Mode = DMA_NORMAL;
	DMA_InitStruct.Init.Priority = DMA_PRIORITY_LOW;
	DMA_InitStruct.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	DMA_InitStruct.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	DMA_InitStruct.Init.MemBurst = DMA_MBURST_SINGLE;
	DMA_InitStruct.Init.PeriphBurst = DMA_PBURST_SINGLE;
	DMA_InitStruct.Init.MemInc = DMA_MINC_DISABLE;
	DMA_InitStruct.Init.Channel = Settings->TX_Channel;
	DMA_InitStruct.Init.Direction = DMA_MEMORY_TO_PERIPH;
	
	/* Set dummy memory to value we specify */
	Settings->Dummy16 = value;
	
	/* Deinit first TX stream */
	TM_DMA_ClearFlag(Settings->TX_Stream, DMA_FLAG_ALL);
	
	/* Init TX stream */
	TM_DMA_Init(Settings->TX_Stream, &DMA_InitStruct);
	
	/* Start TX stream */
	TM_DMA_Start(&DMA_InitStruct, (uint32_t) &Settings->Dummy16, (uint32_t) &I2Cx->DR, count);
	
	/* Enable I2C TX DMA */
	I2Cx->CR2 |= I2C_CR2_DMAEN;
	
	/* Return OK */
	return 1;
}
/**
 * @brief  Check I2C busy status
 */
#define I2C_IS_BUSY(I2Cx)                   (((I2Cx)->SR1 & (I2C_SR1_TXE | I2C_SR1_RXNE)) == 0)


uint8_t TM_I2C_DMA_Transmitting(I2C_TypeDef* I2Cx) {
	/* Get I2C settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Check if TX or RX DMA are working */
	return (
		Settings->RX_Stream->NDTR || /*!< RX is working */
		Settings->TX_Stream->NDTR || /*!< TX is working */
		I2C_IS_BUSY(I2Cx)            /*!< I2C is busy */
	);
}

DMA_Stream_TypeDef* TM_I2C_DMA_GetStreamTX(I2C_TypeDef* I2Cx) {
	/* Return pointer to TX stream */
	return TM_I2C_DMA_INT_GetSettings(I2Cx)->TX_Stream;
}

DMA_Stream_TypeDef* TM_I2C_DMA_GetStreamRX(I2C_TypeDef* I2Cx) {
	/* Return pointer to TX stream */
	return TM_I2C_DMA_INT_GetSettings(I2Cx)->RX_Stream;
}

void TM_I2C_DMA_EnableInterrupts(I2C_TypeDef* I2Cx) {
	/* Get I2C settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Enable interrupts for TX and RX streams */
	TM_DMA_EnableInterrupts(Settings->TX_Stream);
	TM_DMA_EnableInterrupts(Settings->RX_Stream);
}

void TM_I2C_DMA_DisableInterrupts(I2C_TypeDef* I2Cx) {
	/* Get I2C settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	/* Enable interrupts for TX and RX streams */
	TM_DMA_DisableInterrupts(Settings->TX_Stream);
	TM_DMA_DisableInterrupts(Settings->RX_Stream);
}

/* Private functions */
static TM_I2C_DMA_INT_t* TM_I2C_DMA_INT_GetSettings(I2C_TypeDef* I2Cx) {
	TM_I2C_DMA_INT_t* result;
#ifdef I2C1
	if (I2Cx == I2C1) {
		result = &I2C1_DMA_INT;
	}
#endif
#ifdef I2C2
	if (I2Cx == I2C2) {
		result = &I2C2_DMA_INT;
	}
#endif
#ifdef I2C3
	if (I2Cx == I2C3) {
		result = &I2C3_DMA_INT;
	}
#endif
#ifdef I2C4
	if (I2Cx == I2C4) {
		result = &I2C4_DMA_INT;
	}
#endif
#ifdef I2C5
	if (I2Cx == I2C5) {
		result = &I2C5_DMA_INT;
	}
#endif
#ifdef I2C6
	if (I2Cx == I2C6) {
		result = &I2C6_DMA_INT;
	}
#endif

	/* Return */
	return result;
}


// ********************************************************************************************************************//
// Author: Ramiro Vera
// FIUBA
// This is a migration of old libraries stm32f4xx_dma.c and stm32f4xx_i2c.c to try to fix tm_stm32_i2c_dma.c



/**
  * @brief  Enables or disables the specified DMAy Streamx.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *         to 7 to select the DMA Stream.
  * @param  NewState: new state of the DMAy Streamx. 
  *          This parameter can be: ENABLE or DISABLE.
  *
  * @note  This function may be used to perform Pause-Resume operation. When a
  *        transfer is ongoing, calling this function to disable the Stream will
  *        cause the transfer to be paused. All configuration registers and the
  *        number of remaining data will be preserved. When calling again this
  *        function to re-enable the Stream, the transfer will be resumed from
  *        the point where it was paused.          
  *    
  * @note  After configuring the DMA Stream (DMA_Init() function) and enabling the
  *        stream, it is recommended to check (or wait until) the DMA Stream is
  *        effectively enabled. A Stream may remain disabled if a configuration 
  *        parameter is wrong.
  *        After disabling a DMA Stream, it is also recommended to check (or wait
  *        until) the DMA Stream is effectively disabled. If a Stream is disabled 
  *        while a data transfer is ongoing, the current data will be transferred
  *        and the Stream will be effectively disabled only after the transfer of
  *        this single data is finished.            
  *    
  * @retval None
  */
void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected DMAy Streamx by setting EN bit */
    DMAy_Streamx->CR |= (uint32_t)DMA_SxCR_EN;
  }
  else
  {
    /* Disable the selected DMAy Streamx by clearing EN bit */
    DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
  }
}
/*
 ===============================================================================
                           Data Counter functions
 ===============================================================================  

  This subsection provides function allowing to configure and read the buffer size
  (number of data to be transferred). 

  The DMA data counter can be written only when the DMA Stream is disabled 
  (ie. after transfer complete event).

  The following function can be used to write the Stream data counter value:
    - void DMA_SetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t Counter);

@note It is advised to use this function rather than DMA_Init() in situations where
      only the Data buffer needs to be reloaded.

@note If the Source and Destination Data Sizes are different, then the value written in
      data counter, expressing the number of transfers, is relative to the number of 
      transfers from the Peripheral point of view.
      ie. If Memory data size is Word, Peripheral data size is Half-Words, then the value
      to be configured in the data counter is the number of Half-Words to be transferred
      from/to the peripheral.

  The DMA data counter can be read to indicate the number of remaining transfers for
  the relative DMA Stream. This counter is decremented at the end of each data 
  transfer and when the transfer is complete: 
   - If Normal mode is selected: the counter is set to 0.
   - If Circular mode is selected: the counter is reloaded with the initial value
     (configured before enabling the DMA Stream)
   
  The following function can be used to read the Stream data counter value:
     - uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx);

@endverbatim
  * @{
  */

/**
  * @brief  Writes the number of data units to be transferred on the DMAy Streamx.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @param  Counter: Number of data units to be transferred (from 0 to 65535) 
  *          Number of data items depends only on the Peripheral data format.
  *            
  * @note   If Peripheral data format is Bytes: number of data units is equal 
  *         to total number of bytes to be transferred.
  *           
  * @note   If Peripheral data format is Half-Word: number of data units is  
  *         equal to total number of bytes to be transferred / 2.
  *           
  * @note   If Peripheral data format is Word: number of data units is equal 
  *         to total  number of bytes to be transferred / 4.
  *      
  * @note   In Memory-to-Memory transfer mode, the memory buffer pointed by 
  *         DMAy_SxPAR register is considered as Peripheral.
  *      
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
void DMA_SetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t Counter)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));

  /* Write the number of data units to be transferred */
  DMAy_Streamx->NDTR = (uint16_t)Counter;
}

/**
  * @brief  Specifies that the next DMA transfer is the last one.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C DMA last transfer.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */

void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Next DMA transfer is the last transfer */
    I2Cx->CR2 |= I2C_CR2_LAST;
  }
  else
  {
    /* Next DMA transfer is not the last transfer */
    I2Cx->CR2 &= (uint16_t)~((uint16_t)I2C_CR2_LAST);
  }
}


void RV_I2C_DMA_Read(I2C_TypeDef* I2Cx, uint8_t slaveAddr, uint8_t readAddr, uint8_t dataToSend )  
{
		/* Get I2C settings */
	TM_I2C_DMA_INT_t* Settings = TM_I2C_DMA_INT_GetSettings(I2Cx);
	
	DMA_Stream_TypeDef* Stream = Settings->RX_Stream;
	
// /* Disable DMA channel*/
	DMA_Cmd( Stream , DISABLE);

//  /* Set current data number again to 14 for MPu6050, only possible after disabling the DMA channel */
  DMA_SetCurrDataCounter(Stream, dataToSend);

//  /* While the bus is busy */
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
//while(		I2C_IS_BUSY(I2Cx)  );          /*!< I2C is busy */    // REVISAR!!!!! 
	
//  /* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2Cx, ENABLE);                    //Note this one, very important

//  /* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);

//  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

//  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter); 

//  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

//  /* Clear EV6 by setting again the PE bit */
 I2C_Cmd(I2Cx, ENABLE);

//  /* Send the MPU6050's internal address to write to */
 I2C_SendData(I2Cx, readAddr);

//  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//  /* Send STRAT condition a second time */
  I2C_GenerateSTART(I2Cx, ENABLE);

//  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

//  /* Send MPU6050 address for read */
I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);

//  /* Test on EV6 and clear it */
 while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

//  /* Start DMA to receive data from I2C */
 DMA_Cmd( Stream, ENABLE);
 I2C_DMACmd (I2Cx, ENABLE);

//  // When the data transmission is complete, it will automatically jump to DMA interrupt routine to finish the rest.
//  //now go back to the main routine
}


