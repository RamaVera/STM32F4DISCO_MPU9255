/**
 * Keil project example for LEDS and buttons on evaluation boards
 *
 * Before you start, select your target, on the right of the "Load" button
 *
 * @author    Tilen MAJERLE
 * @email     tilen@majerle.eu
 * @website   http://stm32f4-discovery.net
 * @ide       Keil uVision 5
 * @conf      PLL parameters are set in "Options for Target" -> "C/C++" -> "Defines"
 * @packs     STM32F4xx/STM32F7xx Keil packs are requred with HAL driver support
 * @stdperiph STM32F4xx/STM32F7xx HAL drivers required
 */
/* Include core modules */
#include "stm32fxxx_hal.h"
/* Include my libraries here */
#include <stdbool.h>
#include "defines.h"
#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_usb_device.h"
#include "tm_stm32_usb_device_cdc.h"
#include "tm_stm32_i2c.h"
#include "tm_stm32_i2c_dma.h"
#include "tm_stm32_exti.h"
#include "tm_stm32_mpu9250.h"

void TM_USBD_CDC_PutDouble(TM_USB_t USB_Mode, double x, int p);
void TM_USBD_CDC_PutLong(TM_USB_t USB_Mode,long x);
void TM_DMA_TransferCompleteHandler(DMA_Stream_TypeDef* DMA_Stream) ;
void TM_DMA_TransferErrorHandler(DMA_Stream_TypeDef* DMA_Stream);


/* USB CDC settings */
TM_USBD_CDC_Settings_t USB_FS_Settings;

/* MPU6050 working structure */
TM_MPU9250_t MPU9250;

#define DATA_SIZE  6

/* Character value */
int counter;
int counter2=0;
uint8_t Rx_Buff[DATA_SIZE];
static bool dataOk=false;
bool oneTime=false;
bool firstTime=true;

int main(void) 
{
	/* Init system clock for maximum system speed */
	TM_RCC_InitSystem();
	
	/* Init HAL layer */
	HAL_Init();
	
	/* Init leds */
	TM_DISCO_LedInit();
	
	/* Init delay */
	TM_DELAY_Init();
	
	/* Init button */
	TM_DISCO_ButtonInit();
	
	
	TM_MPU9250_Result_t state;
	do{
			/* Try to init MPU9250, device address is 0xD0, AD0 pin is set to low */
			state = TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0);
		  TM_DISCO_LedOff(LED_BLUE);
			}
			while(state != TM_MPU9250_Result_Ok );
			
	/* Blue LED on */
	TM_DISCO_LedOn(LED_BLUE);
	
	//	TM_GPIO_Init(GPIOB, GPIO_Pin_7, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Low);
		/* Attach EXTI pin, enable both edges because of different boards support */
	if (TM_EXTI_Attach(GPIOB, GPIO_PIN_7, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok) {
		/* Turn on green LED */
		TM_DISCO_LedOn(LED_ORANGE);
		TM_MPU9250_DataReady(&MPU9250);
	} else {
		/* Turn on RED led */
		TM_DISCO_LedOff(LED_ORANGE);
		while(1);
	}
	

	/* Init USB peripheral */
	TM_USB_Init();
	
	/* Init VCP on FS and HS ports.. */
	TM_USBD_CDC_Init(TM_USB_FS);
	//TM_USBD_CDC_Init(TM_USB_HS);
	
	/* ..or use single call for both modes */
	//TM_USBD_CDC_Init(TM_USB_Both);
	
	/* Start USB device mode on FS and HS ports.. */
	TM_USBD_Start(TM_USB_FS);
	//TM_USBD_Start(TM_USB_HS);
	
	/* .. or use single call for both modes */
	//TM_USBD_Start(TM_USB_Both);
	

	/* Init SPI DMA */
	TM_I2C_DMA_Init(MPU9250_I2C);
	/* Enable interrupts */
	TM_I2C_DMA_DisableInterrupts(MPU9250_I2C);
	
		
	while (1)
	{
		/* Process USB CDC device, send remaining data if needed */
		/* It is better if you call this in periodic timer, like each ms in SYSTICK handler */
		TM_USBD_CDC_Process(TM_USB_Both);
		TM_USBD_CDC_Puts(TM_USB_FS, ".");

		/* Check if device is ready, if drivers are installed if needed on FS port */
		if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok ) 
		{
			TM_DISCO_LedOn(LED_GREEN);
			if (oneTime ==false){
			TM_I2C_DMA_Transmit(MPU9250_I2C, NULL, Rx_Buff, DATA_SIZE );
			TM_I2C_DMA_EnableInterrupts(MPU9250_I2C);
			oneTime=true;
			}
		} else {
			TM_DISCO_LedOff(LED_GREEN);
		}
		
				
		/* Check if user has changed parameter for COM port */
		TM_USBD_CDC_GetSettings(TM_USB_FS, &USB_FS_Settings);
		
		/* Check if updated */
		if (USB_FS_Settings.Updated) 
		{
			/* Update settings for UART here if needed */
			TM_USBD_CDC_Puts(TM_USB_FS, "USB FS settings changed!\n");
		}
	
//	if(!(TM_I2C_DMA_Transmitting(MPU9250_I2C)))		{	
//		if( TM_MPU9250_DataReady(&MPU9250)== TM_MPU9250_Result_Ok)
//	{
	
		if (dataOk == true)
  	{
			dataOk=false;
			/* Read everything from device */
		static int print=0;	
//			if( print < 3){
//				TM_MPU9250_ReadAcce(&MPU9250);
//			}else {
//				TM_MPU9250_ReadGyro(&MPU9250);
//			}
			
			switch(print++)	
			{
				case 1:
								TM_USBD_CDC_Puts(TM_USB_FS, "\n \r Aceleracion en x: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS, MPU9250.Ax , 5);
								break;
				case 2:		
								TM_USBD_CDC_Puts(TM_USB_FS, "\n \r Aceleracion en y: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS,  MPU9250.Ay, 5);
								break;
				case 3:
								TM_USBD_CDC_Puts(TM_USB_FS, "\n \r Aceleracion en z: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS, MPU9250.Az, 5);
								break;
				case 4: 
//								TM_USBD_CDC_Puts(TM_USB_FS, "\n \r Velocidad en x: ");	
//								TM_USBD_CDC_PutDouble(TM_USB_FS,  MPU9250.Gx, 5);
//								break;
				case 5:
//								TM_USBD_CDC_Puts(TM_USB_FS, "\n \r Velocidad en y: ");	
//								TM_USBD_CDC_PutDouble(TM_USB_FS,  MPU9250.Gy, 5);
//								break;
				case 6:
//								TM_USBD_CDC_Puts(TM_USB_FS, "\n \r Velocidad en z: ");	
//								TM_USBD_CDC_PutDouble(TM_USB_FS, MPU9250.Gz, 5);
//								break;
				case 8:
								print = 1;
								break;
				default:
								print = 1;
								break;
			}
			TM_DISCO_LedOff(LED_BLUE);
			TM_I2C_DMA_Transmit(MPU9250_I2C, NULL, Rx_Buff, DATA_SIZE );
//		/* If button pressed */
//		if (TM_DISCO_ButtonPressed()) {
//			/* Turn on ALL leds */
//			TM_DISCO_LedOn(LED_ALL);
//		} else {
//			/* Turn off ALL leds */
//			TM_DISCO_LedOff(LED_ALL);
//		}
		}
	}
}

/*------------------------------------------------------------------*/
/* DMA transfer complete callback */
void TM_DMA_TransferCompleteHandler(DMA_Stream_TypeDef* DMA_Stream) {

  	TM_DISCO_LedToggle(LED_RED);
//	 /* Clear transmission complete flag */
//    TM_DMA_ClearFlag(TM_I2C_DMA_GetStreamRX(MPU9250_I2C) ,DMA_FLAG_TCIF);

//    I2C_DMACmd(MPU9250_I2C, DISABLE);
////    /* Send I2C1 STOP Condition */
//    I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
////    /* Disable DMA channel*/
//    DMA_Cmd(TM_I2C_DMA_GetStreamRX(MPU9250_I2C), DISABLE);

		MPU9250.Ax_Raw = ((int16_t)Rx_Buff[0] << 8) | Rx_Buff[1];
		MPU9250.Ay_Raw = ((int16_t)Rx_Buff[2] << 8) | Rx_Buff[3];  
		MPU9250.Az_Raw = ((int16_t)Rx_Buff[4] << 8) | Rx_Buff[5];

		MPU9250.Ax = (float)MPU9250.Ax_Raw * MPU9250.AMult;
		MPU9250.Ay = (float)MPU9250.Ay_Raw * MPU9250.AMult;
		MPU9250.Az = (float)MPU9250.Az_Raw * MPU9250.AMult;
			
		dataOk = true;
	  TM_DMA_ClearFlags(TM_I2C_DMA_GetStreamRX(MPU9250_I2C) );

}

void TM_DMA_TransferErrorHandler(DMA_Stream_TypeDef* DMA_Stream) {
	
	TM_DISCO_LedOff(LED_ALL);
	
}

/* Handle all EXTI lines */
void TM_EXTI_Handler(uint16_t GPIO_Pin) {
	/* Check proper line */
//	if (GPIO_Pin == GPIO_Pin_7) 
//	{
		if( firstTime == true){ TM_DISCO_LedToggle(LED_ORANGE); firstTime=false;}
		counter2++;
		TM_MPU9250_DataReady(&MPU9250);
		if( counter2 == 100)
		{
			TM_DISCO_LedToggle(LED_ORANGE);
			counter2=0;
		}
		//RV_I2C_DMA_Read(MPU9250_I2C, MPU9250_I2C_ADDR, ACCEL_XOUT_H, DATA_SIZE );  
		
	//			/* Toggle LEDs if interrupt on button line happens */
	//			TM_DISCO_LedOff(LED_ORANGE);
//	}
}




/*------------------------------------------------------------------*/
void TM_USBD_CDC_PutLong(TM_USB_t USB_Mode,long x)
{
    if(x < 0)
    {
        TM_USBD_CDC_Putc(USB_Mode,'-');
        x = -x;
    }
    if (x >= 10) 
    {
        TM_USBD_CDC_PutLong(USB_Mode,x/10);
    }
    TM_USBD_CDC_Putc(USB_Mode,x % 10+'0');
}

void TM_USBD_CDC_PutDouble(TM_USB_t USB_Mode, double x, int p)
{
    long d;
    if (x<0) {
        TM_USBD_CDC_Putc(USB_Mode,'-');
        x=-x;
    }
    d = x;
    TM_USBD_CDC_PutLong(USB_Mode, d );
    TM_USBD_CDC_Putc(USB_Mode,'.');
    while (p--) {
        x = (x - d) * 10;
        d = x;
        TM_USBD_CDC_Putc(USB_Mode,'0'+d);
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


