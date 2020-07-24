/*******************************************************************************
 * @file    IKS01A2.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    01.01.2019
 *          
 * @brief   Interfacing X-NUCLEO-IKS01A2 motion MEMS and environmental
 *          sensor expansion board.
 * @note    
 *
@verbatim
Copyright (C) Almohandes.org, 2019

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
*******************************************************************************/

/* Includes */
#include <stddef.h>
#include "rv_stm32_dma.h"
/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup IKS01A2
 * @brief
 * @{
 */

/**
 * @defgroup IKS01A2_private_typedefs
 * @{
 */

/**
 * @brief   IKS01A2 DMA data ready flag
 */
typedef enum IKS01A2_DMAStatus
{
  IKS01A2_DMA_NOT_FINISHED,
  IKS01A2_DMA_RX_FINISHED,
  IKS01A2_DMA_TX_FINISHED,
} IKS01A2_DMAStatusType;

typedef struct
{
  float x0;
  float y0;
  float x1;
  float y1;
} Linear_Interpolation_t;

/**
 * @}
 */

/**
 * @defgroup IKS01A2_private_defines
 * @{
 */
/**
 * @brief   AF4 pin 8, 9 masks
 */
#define GPIO_AFRH_AFRH8                      ((uint32_t) 0x0000000F)
#define GPIO_AFRH_AFRH8_AF4                  ((uint32_t) 0x00080004)

#define GPIO_AFRH_AFRH9                      ((uint32_t) 0x000000F0)
#define GPIO_AFRH_AFRH9_AF4                  ((uint32_t) 0x00080040)

/**
 * @brief   Maximum DMA TX/RX buffer length
 */
#define MAX_BUFFER_LENGTH                    ((uint8_t) 20u)

/**
 * @brief   HTS221 relative humidity and temperature
 */
/* HTS221 slave address */
#define HTS221_SLAVE_ADDRESS                  ((uint8_t) 0xBE)

/* HTS221 registers address */
#define HTS221_WHO_AM_I_ADD                   ((uint8_t) 0x0F)
#define HTS221_CTRL_REG1_ADD                  ((uint8_t) 0x20)

#define HTS221_HUMIDITY_OUT_L                 ((uint8_t) 0x28)
#define HTS221_HUMIDITY_OUT_H                 ((uint8_t) 0x29)
#define HTS221_TEMP_OUT_L                     ((uint8_t) 0x2A)
#define HTS221_TEMP_OUT_H                     ((uint8_t) 0x2B)

/* Calibration registers start address */
#define HTS221_CALI_REGS_START                ((uint8_t) 0x30)

/* HTS221 registers values */
#define HTS221_WHO_AM_I_VAL                   ((uint8_t) 0xBC)

/* Set active mode,
 * Disable block data update when reading,
 * Set Output data rate 12.5 Hz */
#define HTS221_CTRL_REG1_VAL                  ((uint8_t) 0x87)

/**
 * @}
 */

/**
 * @defgroup IKS01A2_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup IKS01A2_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup IKS01A2_private_variables
 * @{
 */

/**
 * @brief   DMA TX/RX buffers
 */
static uint8_t RxDMABuffer[MAX_BUFFER_LENGTH];
static uint8_t TxDMABuffer[MAX_BUFFER_LENGTH];

static uint8_t RxDMACaliDataBuffer[MAX_BUFFER_LENGTH];

/**
 * @brief   IKS01A2 DMA Status flag
 */
static IKS01A2_DMAStatusType DMAStatus = IKS01A2_DMA_NOT_FINISHED;

/**
 * @brief   Temperature
 */
static float Temperature = 0.0;

/**
 * @}
 */

/**
 * @defgroup IKS01A2_private_function_prototypes
 * @{
 */
static void DMA_Transmit(const uint8_t * pBuffer, uint8_t size);
static void DMA_Receive(const uint8_t * pBuffer, uint8_t size);
static void IKS01A2_ReadSensorReg(uint8_t SensorAddr, uint8_t ReadAddr,
    uint8_t * pReadBuffer, uint16_t NumByteToRead);
static void IKS01A2_WriteSensorReg(uint8_t SensorAddr,
    uint8_t * pWriteBuffer, uint16_t NumByteToWrite);
static void IKS01A2_GPIO_Config(void);
static void IKS01A2_I2C3_Config(void);
static void IKS01A2_DMA_RX_Config(void);
static void IKS01A2_DMA_TX_Config(void);
static void IKS01A2_Sensors_Init(void);

/**
 * @}
 */

/**
 * @defgroup IKS01A2_private_functions
 * @{
 */

/**
 * @brief   DMA data transmit
 * @note    I2C3_TX -> DMA1, Stream 4, Channel 3
 * @param   pBuffer, size
 * @retval  None
 */
static void DMA_Transmit(const uint8_t * pBuffer, uint8_t size)
{
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    /* Wait until DMA1_Stream4 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream4->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA1_Stream4->M0AR = (uint32_t)pBuffer;

    /* Set number of data items */
    DMA1_Stream4->NDTR = size;

    /* Clear all interrupt flags */
    DMA1->HIFCR = (DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4
        | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4);

    /* Reset DMA flag */
    DMAStatus = IKS01A2_DMA_NOT_FINISHED;

    /* Enable DMA1_Stream4 */
    DMA1_Stream4->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   DMA data receive
 * @note    I2C3_RX -> DMA1, Stream 2, Channel 3
 * @param   pBuffer, size
 * @retval  None
 */
static void DMA_Receive(const uint8_t * pBuffer, uint8_t size)
{
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    /* Wait until DMA1_Stream2 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream2->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA1_Stream2->M0AR = (uint32_t)pBuffer;

    /* Set number of data items */
    DMA1_Stream2->NDTR = size;

    /* Clear all interrupt flags */
    DMA1->LIFCR = (DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2
        | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2);

    /* Reset DMA flag */
    DMAStatus = IKS01A2_DMA_NOT_FINISHED;

    /* Enable DMA1_Stream2 */
    DMA1_Stream2->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   Read register data
 * @note
 * @param   SensorAddr, ReadAddr, pReadBuffer, NumByteToRead
 * @retval  None
 */
static void IKS01A2_ReadSensorReg(uint8_t SensorAddr, uint8_t ReadAddr,
    uint8_t * pReadBuffer, uint16_t NumByteToRead)
{
  /* Generate START */
  I2C3->CR1 |= I2C_CR1_START;

  /* Wait SB flag is set */
  while(I2C_SR1_SB != (I2C_SR1_SB & I2C3->SR1))
  {
    /* Do nothing */
  }

  /* Read SR1 */
  (void)I2C3->SR1;

  /* Send slave address with write */
  I2C3->DR = (uint16_t) SensorAddr;

  /* Wait ADDR flag is set */
  while(I2C_SR1_ADDR != (I2C_SR1_ADDR & I2C3->SR1))
  {
    /* Do nothing */
  }

  /* Read SR1 */
  (void)I2C3->SR1;

  /* Read SR2 */
  (void)I2C3->SR2;

  /* Wait TXE flag is set */
  while(I2C_SR1_TXE != (I2C_SR1_TXE & I2C3->SR1))
  {
    /* Do nothing */
  }

  if(2 <= NumByteToRead)
  {
    /* Acknowledge enable */
    I2C3->CR1 |= I2C_CR1_ACK;

    /* Send register address to read with increment */
    I2C3->DR = (uint16_t) (ReadAddr | (uint8_t)0x80);
  }
  else
  {
    /* Acknowledge disable */
    I2C3->CR1 &= ~I2C_CR1_ACK;

    /* Send register address to read (single) */
    I2C3->DR = (uint16_t) ReadAddr;
  }



  /* Wait BTF flag is set */
  while(I2C_SR1_BTF != (I2C_SR1_BTF & I2C3->SR1))
  {
    /* Do nothing */
  }

  /* Generate ReSTART */
  I2C3->CR1 |= I2C_CR1_START;

  /* Wait SB flag is set */
  while(I2C_SR1_SB != (I2C_SR1_SB & I2C3->SR1))
  {
    /* Do nothing */
  }

  /* Read SR1 */
  (void)I2C3->SR1;

  /* Send slave address with read */
  I2C3->DR = (uint16_t) (SensorAddr | (uint8_t)0x01);

  /* Wait ADDR flag is set */
  while(I2C_SR1_ADDR != (I2C_SR1_ADDR & I2C3->SR1))
  {
    /* Do nothing */
  }

  /* Start DMA */
  DMA_Receive(pReadBuffer, NumByteToRead);

  /* Read SR1 */
  (void)I2C3->SR1;

  /* Read SR2 */
  (void)I2C3->SR2;
}

/**
 * @brief   Write register data
 * @note
 * @param   SensorAddr, pWriteBuffer, NumByteToWrite
 * @retval  None
 */
static void IKS01A2_WriteSensorReg(uint8_t SensorAddr,
    uint8_t * pWriteBuffer, uint16_t NumByteToWrite)
{
  /* Generate START */
  I2C3->CR1 |= I2C_CR1_START;

  /* Wait SB flag is set */
  while(I2C_SR1_SB != (I2C_SR1_SB & I2C3->SR1))
  {
    /* Do nothing */
  }

  /* Read SR1 */
  (void)I2C3->SR1;

  /* Send slave address with write */
  I2C3->DR = (uint16_t) SensorAddr;

  /* Wait ADDR flag is set */
  while(I2C_SR1_ADDR != (I2C_SR1_ADDR & I2C3->SR1))
  {
    /* Do nothing */
  }

  /* Start DMA */
  DMA_Transmit(pWriteBuffer, NumByteToWrite);

  /* Read SR1 */
  (void)I2C3->SR1;

  /* Read SR2 */
  (void)I2C3->SR2;
}

/**
 * @}
 */

/**
 * @defgroup IKS01A2_exported_functions
 * @{
 */

/**
 * @brief   GPIO Configure, PA8 -> I2C3_SCL, PC9 -> I2C3_SDA
 * @note
 * @param   None
 * @retval  None
 */
static void IKS01A2_GPIO_Config(void)
{
  /* Enable port A and C clocks */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN);

  /* Select Alternate function mode */
  GPIOA->MODER &= ~GPIO_MODER_MODER8;
  GPIOA->MODER |= GPIO_MODER_MODER8_1;
  GPIOC->MODER &= ~GPIO_MODER_MODER9;
  GPIOC->MODER |= GPIO_MODER_MODER9_1;

  /* Select output type open drain */
  GPIOA->OTYPER |= GPIO_OTYPER_OT_8;
  GPIOC->OTYPER |= GPIO_OTYPER_OT_9;

  /* Select output speed very high */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR8;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR8_1;
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR9;
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR9_1;

  /* Select pull-up */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0;

  /* Select AF4 */
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH8;
  GPIOA->AFR[1] |= GPIO_AFRH_AFRH8_AF4;
  GPIOC->AFR[1] &= ~GPIO_AFRH_AFRH9;
  GPIOC->AFR[1] |= GPIO_AFRH_AFRH9_AF4;
}

/**
 * @brief   I2C3 Master Configuration
 * @note
 * @param   None
 * @retval  None
 */
static void IKS01A2_I2C3_Config(void)
{
  /* Enable I2C3 clock */
  RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

  /* Make sure the peripheral is disabled for clock configuration */
  I2C3->CR1 &= ~I2C_CR1_PE;

  /* Select Peripheral clock frequency, APB1 = 45 MHz */
  I2C3->CR2 &= ~I2C_CR2_FREQ;
  I2C3->CR2 |= (I2C_CR2_FREQ_0 | I2C_CR2_FREQ_2
      | I2C_CR2_FREQ_3 | I2C_CR2_FREQ_5);

  /* Select fast mode */
  I2C3->CCR |= I2C_CCR_FS;

  /* Set duty cycle to reach 400 KHz */
  I2C3->CCR |= I2C_CCR_DUTY;

  /* CCR Clock configuration for Fast I2C mode
   * T_high = 9 * CCR * T_PCLK1
   * T_low = 16 * CCR * T_PCLK1
   *
   * I2C_Clock_Period = T_high + T_low
   *  = CCR * T_PCLK1 * (9 + 16) = CCR * T_PCLK1 * 25
   *
   *  CCR = I2C_Clock_Period / (T_PCLK1 * 25)
   *      = PCLK1 / (I2C_Clock_Speed * 25)
   *      = 45000000 / (400000 * 25) = 4.5
   *
   * I2C_Clock_Speed = PCLK1 / (CCR * 25)
   *                 = 45000000 / (4 * 25) = 450000 NOK
   *                 = 45000000 / (5 * 25) = 360000 OK
   *
   * So PCLK1 must be a multiple of 10MHz to reach the
   * 400 kHz maximum I²C Fm mode clock.
   *
   * T_high = (9 * 5) / 45000000  = 1us OK (in standard should be > 0.6us )
   * T_low  = (25 * 5) / 45000000 = 2.7us OK (in standard should be > 1.3us)
   * Check UM10204 Table 10. */
  I2C3->CCR &= ~I2C_CCR_CCR;
  I2C3->CCR |= (uint16_t) 5u;

  /* Rise time configuration
   * In UM10204 Table 10. max rise time for Fast-mode is 300ns
   * TRISE = (300ns / 22.22ns) + 1 = 14.5
   *
   * if we take 14 then;
   * Rise_Time = (14 - 1) * 22.22ns = 288.86 (is OK) */
  I2C3->TRISE &= ~I2C_TRISE_TRISE;
  I2C3->TRISE |= (uint8_t) 14u;

  /* Clock stretching enabled */
  I2C3->CR1 &= ~I2C_CR1_NOSTRETCH;

  /* General call disabled */
  I2C3->CR1 &= ~I2C_CR1_ENGC;

  /* Peripheral enable */
  I2C3->CR1 |= I2C_CR1_PE;

  /* Next DMA EOT is the last transfer */
  I2C3->CR2 |= I2C_CR2_LAST;

  /* DMA request enabled when TxE=1 or RxNE =1 */
  I2C3->CR2 |= I2C_CR2_DMAEN;

  /* TxE = 1 or RxNE = 1 does not generate any interrupt. */
  I2C3->CR2 &= ~I2C_CR2_ITBUFEN;

  /* Event interrupt enabled */
  I2C3->CR2 |= I2C_CR2_ITEVTEN;

  /* Error interrupt enabled */
  I2C3->CR2 |= I2C_CR2_ITERREN;
}

/**
 * @brief   DMA Configuration, I2C3_RX -> DMA1, Stream 2, Channel 3
 * @note
 * @param   None
 * @retval  None
 */
static void IKS01A2_DMA_RX_Config(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Make sure that the DMA1_Stream2 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream2->CR))
  {
    /* DMA1_Stream2 is enabled, shall be disabled first */
    DMA1_Stream2->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream2->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, DMA1_Stream2 is not enabled */
  }

  /* Select the DMA channel 3 in CHSEL[2:0] in the DMA_SxCR */
  DMA1_Stream2->CR &= ~DMA_SxCR_CHSEL;
  DMA1_Stream2->CR |= (DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1);

  /* Select stream priority very high */
  DMA1_Stream2->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction peripheral-to-memory */
  DMA1_Stream2->CR &= ~DMA_SxCR_DIR;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA1_Stream2->CR &= ~DMA_SxCR_MSIZE;
  DMA1_Stream2->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA1_Stream2->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA1_Stream2->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA1_Stream2->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA1_Stream2->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA1_Stream2->PAR = (uint32_t)&I2C3->DR;
}

/**
 * @brief   DMA Configuration, I2C3_TX -> DMA1, Stream 4, Channel 3
 * @note
 * @param   None
 * @retval  None
 */
static void IKS01A2_DMA_TX_Config(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Make sure that the DMA1_Stream4 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream4->CR))
  {
    /* DMA1_Stream4 is enabled, shall be disabled first */
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream4->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, DMA1_Stream4 is not enabled */
  }

  /* Select the DMA channel 3 in CHSEL[2:0] in the DMA_SxCR */
  DMA1_Stream4->CR &= ~DMA_SxCR_CHSEL;
  DMA1_Stream4->CR |= (DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1);

  /* Select stream priority very high */
  DMA1_Stream4->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction memory-to-peripheral */
  DMA1_Stream4->CR &= ~DMA_SxCR_DIR;
  DMA1_Stream4->CR |= DMA_SxCR_DIR_0;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA1_Stream4->CR &= ~DMA_SxCR_MSIZE;
  DMA1_Stream4->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA1_Stream4->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA1_Stream4->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA1_Stream4->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA1_Stream4->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA1_Stream4->PAR = (uint32_t)&I2C3->DR;
}

/**
 * @brief   Sensors Initialization function
 * @note
 * @param   None
 * @retval  None
 */
static void IKS01A2_Sensors_Init(void)
{
  uint8_t InitStatus = 0;

  /* Check HTS221 capacitive digital relative humidity and temperature */
  IKS01A2_ReadSensorReg(HTS221_SLAVE_ADDRESS, HTS221_WHO_AM_I_ADD,
      RxDMABuffer, 1);

  /* Wait DMA finished flag */
  while(!(IKS01A2_DMA_RX_FINISHED == DMAStatus))
  {
    /* Do nothing */
  }

  /* Check HTS221 with expected value */
  if(HTS221_WHO_AM_I_VAL == RxDMABuffer[0])
  {
    /* Do nothing */
  }
  else
  {
    InitStatus = 1;
  }

  /* Check init status */
  if(1 == InitStatus)
  {
    /* Turn on red led */
    GPIO_TurnON_LED(EVAL_RED_LED);
  }
  else
  {
    /* Turn on green led */
    GPIO_TurnON_LED(EVAL_GREEN_LED);
  }

  /* HTS221, read calibration data */
  IKS01A2_ReadSensorReg(HTS221_SLAVE_ADDRESS, HTS221_CALI_REGS_START,
      RxDMACaliDataBuffer, 16);

  /* Wait DMA finished flag */
  while(!(IKS01A2_DMA_RX_FINISHED == DMAStatus))
  {
    /* Do nothing */
  }

  /* Copy calibration data */
  /* Configure HTS221 */
  TxDMABuffer[0] = HTS221_CTRL_REG1_ADD;
  TxDMABuffer[1] = HTS221_CTRL_REG1_VAL;
  IKS01A2_WriteSensorReg(HTS221_SLAVE_ADDRESS, TxDMABuffer, 2);

  /* Wait DMA finished flag */
  while(!(IKS01A2_DMA_TX_FINISHED == DMAStatus))
  {
    /* Do nothing */
  }
}

/**
 * @brief   Configuration function
 * @note
 * @param   None
 * @retval  None
 */
void IKS01A2_Init(void)
{
  IKS01A2_GPIO_Config();
  IKS01A2_I2C3_Config();
  IKS01A2_DMA_RX_Config();
  IKS01A2_DMA_TX_Config();

  IKS01A2_Sensors_Init();
}

/**
 * @brief   IKS01A2 main function
 * @note
 * @param   None
 * @retval  None
 */
void IKS01A2_Main(void)
{
  /* used for linear interpolation */
  Linear_Interpolation_t linInter;
  float Raw_Temperature = 0;
  uint8_t temp_0;
  uint8_t temp_1;
  uint16_t temp;

  /* Read humidity and temperature */
  IKS01A2_ReadSensorReg(HTS221_SLAVE_ADDRESS, HTS221_HUMIDITY_OUT_L,
      RxDMABuffer, 4);

  SysTick_Delay(100);

  /* Wait DMA finished flag */
  while(!(IKS01A2_DMA_RX_FINISHED == DMAStatus))
  {
    /* Do nothing */
  }

  /* Get temperature raw data */
  Raw_Temperature = (float)((RxDMABuffer[3] << 8) | RxDMABuffer[2]);

  /* Prepare data for interpolation */
  linInter.x0 = (float)((int16_t)((RxDMACaliDataBuffer[13] << 8)
      | RxDMACaliDataBuffer[12]));
  linInter.x1 = (float)((int16_t)((RxDMACaliDataBuffer[15] << 8)
      | RxDMACaliDataBuffer[14]));

  temp_0 = RxDMACaliDataBuffer[2];
  temp_1 = RxDMACaliDataBuffer[5] & (uint8_t)0x03;
  temp = (temp_1 << 8) | temp_0;
  linInter.y0 = (float)temp / 8.0;

  temp_0 = RxDMACaliDataBuffer[3];
  temp_1 = (RxDMACaliDataBuffer[5] & (uint8_t)0x0C) >> 2;
  temp = (temp_1 << 8) | temp_0;
  linInter.y1 = (float)temp / 8.0;

  /* Convert to degree using linear interpolation,
   * (y-y0) / (x-x0) = (y1-y0) / (x1-x0)
   *
   * (y-y0) = ((y1-y0) * (x-x0)) / (x1-x0)
   *  y = (((y1-y0) * (x-x0)) / (x1-x0)) + y0 */
  Temperature = (((linInter.y1 - linInter.y0)
      * (Raw_Temperature - linInter.x0)) / (linInter.x1-linInter.x0))
          + linInter.y0;
}

/**
 * @brief   IRQ callback function
 * @note    I2C3_TX -> DMA1, Stream 4, Channel 3
 * @param   None
 * @retval  None
 */
void I2C3_TX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF4 == (DMA_HISR_TCIF4 & DMA1->HISR))
  {
    /* Clear all interrupt flags */
    DMA1->HIFCR = (DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4
        | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4);

    /* Wait BTF flag is set */
    while(I2C_SR1_BTF != (I2C_SR1_BTF & I2C3->SR1))
    {
      /* Do nothing */
    }

    /* DMA finished */
    DMAStatus = IKS01A2_DMA_TX_FINISHED;

    /* Generate STOP */
    I2C3->CR1 |= I2C_CR1_STOP;
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
  }
}

/**
 * @brief   IRQ callback function
 * @note    I2C3_RX -> DMA1, Stream 2, Channel 3
 * @param   None
 * @retval  None
 */
void I2C3_RX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_LISR_TCIF2 == (DMA_LISR_TCIF2 & DMA1->LISR))
  {
    /* Clear all interrupt flags */
    DMA1->LIFCR = (DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2
        | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2);

    /* DMA finished */
    DMAStatus = IKS01A2_DMA_RX_FINISHED;

    /* Generate STOP */
    I2C3->CR1 |= I2C_CR1_STOP;
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
  }
}
/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */
