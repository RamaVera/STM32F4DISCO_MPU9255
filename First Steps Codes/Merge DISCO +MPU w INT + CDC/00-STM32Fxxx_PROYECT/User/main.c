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
#include "tm_stm32_exti.h"
#include "tm_stm32_mpu9250.h"


void TM_USBD_CDC_PutDouble(TM_USB_t USB_Mode, double x, int p);
void TM_USBD_CDC_PutLong(TM_USB_t USB_Mode,long x);

/* USB CDC settings */
TM_USBD_CDC_Settings_t USB_FS_Settings;

/* MPU9250 working structure */
TM_MPU9250_t MPU9250;

/* Character value */

int counter2=0;
bool incomingData = false;

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
	
	//	TM_GPIO_Init(GPIOB, GPIO_Pin_7, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Low);
		/* Attach EXTI pin, enable both edges because of different boards support */
	if (TM_EXTI_Attach(GPIOB, GPIO_PIN_7, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok) {
		/* Turn on green LED */
		TM_DISCO_LedOn(LED_ORANGE);
	} else {
		/* Turn on RED led */
		TM_DISCO_LedOn(LED_RED);
		while(1);
	}
	
	
	TM_MPU9250_Result_t state = TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0);
	/* Try to init MPU9250, device address is 0xD0, AD0 pin is set to low */
	if ( state == TM_MPU9250_Result_Ok) {
		/* Blue LED on */
		TM_DISCO_LedOn(LED_BLUE);
	}else if(state == TM_MPU9250_Result_DeviceNotConnected)
	{
		TM_DISCO_LedOn(LED_RED);
		while(1);
	}

	/* Init USB peripheral */
	TM_USB_Init();
	
	/* Init VCP on FS and HS ports.. */
	TM_USBD_CDC_Init(TM_USB_FS);
	

	/* Start USB device mode on FS and HS ports.. */
	TM_USBD_Start(TM_USB_FS);
	

		
	while (1)
	{
		/* Process USB CDC device, send remaining data if needed */
		/* It is better if you call this in periodic timer, like each ms in SYSTICK handler */
		TM_USBD_CDC_Process(TM_USB_FS);
		
		/* Check if device is ready, if drivers are installed if needed on FS port */
		if (TM_USBD_IsDeviceReady(TM_USB_FS) == TM_USBD_Result_Ok) 
		{
			TM_DISCO_LedOn(LED_GREEN);
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

	if( 	incomingData == true )
	{ 
		incomingData=false;
			/* Read everything from device */
		static int print=0;	
		if( print < 3){
			TM_MPU9250_ReadAcce(&MPU9250);
		}else {
			TM_MPU9250_ReadGyro(&MPU9250);
		}
			
			switch(print++)	
			{
				case 1:
								TM_USBD_CDC_Puts(TM_USB_FS, "\n Aceleracion en x: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS, MPU9250.Ax , 5);
								break;
				case 2:		
								TM_USBD_CDC_Puts(TM_USB_FS, "\n Aceleracion en y: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS,  MPU9250.Ay, 5);
								break;
				case 3:
								TM_USBD_CDC_Puts(TM_USB_FS, "\n Aceleracion en z: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS, MPU9250.Az, 5);
								break;
				case 4: 
								TM_USBD_CDC_Puts(TM_USB_FS, "\n Velocidad en x: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS,  MPU9250.Gx, 5);
								break;
				case 5:
								TM_USBD_CDC_Puts(TM_USB_FS, "\n Velocidad en y: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS,  MPU9250.Gy, 5);
								break;
				case 6:
								TM_USBD_CDC_Puts(TM_USB_FS, "\n Velocidad en z: ");	
								TM_USBD_CDC_PutDouble(TM_USB_FS, MPU9250.Gz, 5);
								break;
				case 8:
								print = 1;
								break;
				default:
								print = 1;
								break;
			}
		}
	}
}

/*------------------------------------------------------------------*/
/* Handle all EXTI lines */
void TM_EXTI_Handler(uint16_t GPIO_Pin) {
	/* Check proper line */
	//TM_DISCO_LedOn(LED_GREEN); //GreenTestPoint for interrupt
	counter2++;

	if( counter2 == 100)
	{
	TM_DISCO_LedToggle(LED_ORANGE);
	counter2=0;  
	}
		
//	if (GPIO_Pin == GPIO_Pin_7) {
//			/* Toggle LEDs if interrupt on button line happens */
//			TM_DISCO_LedOff(LED_ORANGE);

//		}
	incomingData = true;
	TM_MPU9250_DataReady(&MPU9250);
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
