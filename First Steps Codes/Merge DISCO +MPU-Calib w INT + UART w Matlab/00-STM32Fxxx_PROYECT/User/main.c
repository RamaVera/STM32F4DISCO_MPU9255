/**
 * Keil project example for UART peripheral
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
#include <stdbool.h>
#include <math.h>
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_usart.h"
#include "tm_stm32_mpu9250.h"
#include "tm_stm32_exti.h"
#include "tm_stm32_button.h"

#define MAX_CONNECTION_TRY 5
#define PI 3.14159265359f

/* MPU9250 working structure */
TM_MPU9250_t MPU9250;
static bool incomingData = false;
float accelBias[3];
float gyroBias[3];

/* Button pointer */
TM_BUTTON_t* MyButton;
static bool notPrepared = true;

/* Button callback function */
static void BUTTON_Callback(TM_BUTTON_t* ButtonPtr, TM_BUTTON_PressType_t PressType);
void TM_USART_PutLong(USART_TypeDef* USARTx,long x);
void TM_USART_PutDouble(USART_TypeDef* USARTx, double x, int p);

/* Buffer array */
char incomingBuffer[10]; //9


int main(void) {
     register unsigned int i;
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
     
          /*Wake up MPU and Calibrate*/
     TM_MPU9250_BeginI2CCom(&MPU9250, TM_MPU9250_Device_0);
     TM_MPU9250_Reset(&MPU9250);
     //TM_CalibrateMPU9250(&MPU9250,gyroBias,accelBias);
			
     MyButton = TM_BUTTON_Init(DISCO_BUTTON_PORT, DISCO_BUTTON_PIN, DISCO_BUTTON_PRESSED, BUTTON_Callback);
	TM_BUTTON_SetPressTime(MyButton, 30, 2000);
	TM_DISCO_LedOn(LED_ALL);

     while ( notPrepared == true) {
		/* Update all buttons */
		TM_BUTTON_Update();
	}
     
     /*-----------------------------------------------------------------------------------------------*/
     

    /* Try to init MPU9250 */
	if( TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0) ==TM_MPU9250_Result_Ok ) {
          TM_DISCO_LedOn(LED_BLUE);
	} else {
          TM_DISCO_LedOn(LED_RED);
         while (1) ;      
     }

    /* Atach PB7 to MPU9250 interrupt*/
	if (TM_EXTI_Attach(GPIOB, GPIO_PIN_7, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok) {
          TM_EXTI_GPIOEnable(GPIO_PIN_7,TM_EXTI_Disable); // Begin with interrupts disable
		TM_DISCO_LedOn(LED_ORANGE);
	} else {
		TM_DISCO_LedOn(LED_RED);
		while(1);
	}
     
     /* Init USART2, TX: PA3 RX: PA2, 921600 bauds */
	TM_USART_Init(USART2, TM_USART_PinsPack_1, 921600);
 
	/* Try to communicate with Matlab */
     for(i=0 ; i<MAX_CONNECTION_TRY ; i++ )
     {
          while ( TM_USART_BufferEmpty(USART2) == 0 ){ TM_USART_ClearBuffer(USART2);}
          /*Start conversation with Matlab*/
          TM_USART_Puts(USART2, "HelloMatlab \n \r");
          Delayms(500);
          /*Wait for Matlab response*/
          TM_USART_Gets(USART2, incomingBuffer, sizeof(incomingBuffer));
          Delayms(1000);
         // TM_USART_Send(USART2,(uint8_t *) incomingBuffer, sizeof(incomingBuffer) );
         
          if(strcmp( incomingBuffer,"HelloSTM\n") ==0 )
          {
               TM_DISCO_LedOn(LED_GREEN);
               break;
          } else {
               TM_DISCO_LedOn(LED_RED);
          }
     }         
          /* Start external interrupt*/
     TM_EXTI_GPIOEnable(GPIO_PIN_7,TM_EXTI_Enable);
     TM_MPU9250_DataReady(&MPU9250);

	while (1) 
     {
          if( 	incomingData == true )
          { 
               incomingData = false;
               /* Read everything from device */
               TM_MPU9250_ReadAcce(&MPU9250);
               TM_MPU9250_ReadGyro(&MPU9250);
               
//               float norm =sqrt( pow((9.8*MPU9250.Ax),2)+pow((9.8*MPU9250.Ay),2) +pow((MPU9250.Az*9.8),2) );
               float pitch = ((asin(MPU9250.Ay))/PI)*180;
               float roll = ((asin(MPU9250.Ax)/PI)*180);
               float yaw = ((acos(MPU9250.Az)/PI)*180);
               
               TM_USART_PutDouble(USART2,pitch, 5);  TM_USART_Putc(USART2 ,',');
               TM_USART_PutDouble(USART2,roll, 5);     TM_USART_Putc(USART2 ,',');
               TM_USART_PutDouble(USART2,yaw, 5);    TM_USART_Putc(USART2 ,'\n');
//               
               
//               TM_USART_PutDouble(USART2,MPU9250.Ax, 5);    TM_USART_Putc(USART2 ,',');
//               TM_USART_PutDouble(USART2,MPU9250.Ay, 5);    TM_USART_Putc(USART2 ,',');
//               TM_USART_PutDouble(USART2,MPU9250.Az, 5);    TM_USART_Putc(USART2 ,'\n');
//               TM_USART_PutDouble(USART2,MPU9250.Gx, 5);    TM_USART_Putc(USART2 ,',');
//               TM_USART_PutDouble(USART2,MPU9250.Gy, 5);    TM_USART_Putc(USART2 ,',');
//               TM_USART_PutDouble(USART2,MPU9250.Gz, 5);    TM_USART_Putc(USART2,'\n');
               Delayms(100);
               TM_MPU9250_DataReady(&MPU9250);
               
		}  
     }     // end While(1)  
}         // end Main



/*------------------------------------------------------------------*/
/* Handle all EXTI lines */
void TM_EXTI_Handler(uint16_t GPIO_Pin) {
	/* Check proper line */
	//TM_DISCO_LedOn(LED_GREEN); //GreenTestPoint for interrupt
     static int counter = 0;
if (GPIO_Pin == GPIO_Pin_7) 
{
		counter++;
		if( counter == 1)
		{
		TM_DISCO_LedToggle(LED_ORANGE);
		counter=0;  
		}
	}
	incomingData = true;
}

/* Implement handler function */
static void BUTTON_Callback(TM_BUTTON_t* ButtonPtr, TM_BUTTON_PressType_t PressType) {
	/* Normal press detected */
	if (PressType == TM_BUTTON_PressType_Normal) {
		/* Set LEDS ON */
		TM_DISCO_LedOff(LED_ALL);
          notPrepared = false;
	} 
}

/*------------------------------------------------------------------*/
void TM_USART_PutLong(USART_TypeDef* USARTx,long x)
{
    if(x < 0)
    {
        TM_USART_Putc(USARTx,'-');
        x = -x;
    }
    if (x > 9 ) 
    {
        TM_USART_PutLong(USARTx,x/10);
    }
    TM_USART_Putc(USARTx, (x%10) + '0');
}

void TM_USART_PutDouble(USART_TypeDef* USARTx, double x, int p)
{
    long d;
    if (x<0) {
        TM_USART_Putc(USARTx,'-');
        x=-x;
    }
    d = (long )x;
    TM_USART_PutLong(USARTx, d );
    TM_USART_Putc(USARTx,'.');
    while (p--) {
        x = (x - d) * 10;
        d = (long) x;
        TM_USART_Putc(USARTx,'0'+d);
    }
}
