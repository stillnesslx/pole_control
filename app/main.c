/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
//#include "stm32_eval.h"
#include <stdio.h>
#include <math.h>
#include "bsp.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "serial.h"
//#include "comtest.h"
//#include "oled.h"
#include "semphr.h"
//#include "oo_oled.h"
//#include "oledfont.h"  	
#include <string.h>
#include "loopback.h"
//#include "httpServer.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Task priorities. */
#define DI_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define DO_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define LED_TASK_PRIORITY				( tskIDLE_PRIORITY + 6 )
#define ETH_TASK_PRIORITY				( tskIDLE_PRIORITY + 7 )
//#define RS422_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
//#define RIN_TASK_PRIORITY				( tskIDLE_PRIORITY + 5 )
//#define OLED091_TASK_PRIORITY           ( tskIDLE_PRIORITY + 1 )
//#define OLED242_TASK_PRIORITY           ( tskIDLE_PRIORITY + 4 )

/* Baud rate used by the comtest tasks. */
//#define mainCOM_TEST_BAUD_RATE		( 115200 )
/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
//#define mainCOM_TEST_LED			( 3 )
/* Private macro -------------------------------------------------------------*/
//#define DSP_MODE_EN 0
/* Private variables ---------------------------------------------------------*/
//char RxBuffer1[512];
//uint8_t RxCounter1=0;

//u8 hx = 24,hy = 0,hlen = 3,hsize = 32;
//u32 h = 60;
//u8 test_en=0;
//const uint8_t high[3] = {50,100,150};
//SemaphoreHandle_t xSemaphore = NULL;

#define SOCK_TCPS       0
#define PORT_TCPS		5000

//#define DATA_BUF_SIZE   2048 	// defined in loopback.h
//#define DATA_BUF_SIZE   64 	// defined in loopback.h
uint8_t gDATABUF[DATA_BUF_SIZE];

//#define MAX_HTTPSOCK	1

uint8_t tcp_rx_buf[DATA_BUF_SIZE];
//uint8_t TX_BUF[DATA_BUF_SIZE];

/* Private function prototypes -----------------------------------------------*/

void start_di_task(UBaseType_t prio);
void start_do_task(UBaseType_t prio);
void start_led_task(UBaseType_t prio);
//void start_rin_task(UBaseType_t prio);
//void start_rs422_task(UBaseType_t prio);
//void start_oled091_task(UBaseType_t prio);
//void start_oled242_task(UBaseType_t prio);
void start_eth_task(UBaseType_t prio);

static void prvSetupHardware( void );
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

  /* Initialize LEDs, Key Button, LCD and COM port(USART) available on
     STM3210X-EVAL board ******************************************************/

    prvSetupHardware();
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */


  /* Initialize the LCD */


  /* Display message on STM3210X-EVAL LCD *************************************/
  /* Clear the LCD */ 


  /* Retarget the C library printf function to the USARTx, can be USART1 or USART2
     depending on the EVAL board you are using ********************************/


  /* Turn on leds available on STM3210X-EVAL **********************************/


  /* Add your application code here*/
    
    //vSemaphoreCreateBinary( xSemaphore );
    delay_ms(200);
    bsp_init();
    start_do_task(DO_TASK_PRIORITY);
    start_di_task(DI_TASK_PRIORITY);
    start_led_task(LED_TASK_PRIORITY);
    start_eth_task(ETH_TASK_PRIORITY);
    //if( xSemaphore != NULL )
    //{
        //start_rin_task(RIN_TASK_PRIORITY);
        //start_oled091_task(OLED091_TASK_PRIORITY);
    //}
    //start_oled242_task(OLED242_TASK_PRIORITY);
    //start_rs422_task(RS422_TASK_PRIORITY);
    //vAltStartComTestTasks( RS422_TASK_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED);
    /* Start the scheduler. */
    vTaskStartScheduler();
    /* Infinite loop */
    while (1)
    {
    }
}
#define ETH_STACK_SIZE	            ( ( unsigned short ) 256 )
#define LED_STACK_SIZE	            ( ( unsigned short ) 32 )
#define DI_STACK_SIZE	            ( ( unsigned short ) 32 )
#define DO_STACK_SIZE	            ( ( unsigned short ) 32 )
//#define RIN_STACK_SIZE	            ( ( unsigned short ) 56 )
//#define RS422_STACK_SIZE	        ( ( unsigned short ) 128 )
//#define RS422_RECEIVE_STACK_SIZE    ( ( unsigned short ) 128 )
//#define OLED091_STACK_SIZE          ( ( unsigned short ) 256 )
//#define OLED242_STACK_SIZE          ( ( unsigned short ) 256 )

void memcpy_word(u16 *src,u16 *det,u8 len)
{
    while(len--)
    {
        *src++ = *det++;
    }
}

static void di_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        read_di(&di_value);
		vTaskDelay( 10 / portTICK_PERIOD_MS ); /* Delay 1000 ms */
	}
}
static void do_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        //read_di(&di_value);
        write_do(do_value);
		vTaskDelay( 10 / portTICK_PERIOD_MS ); /* Delay 1000 ms */
	}
}
static void led_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        static uint8_t led = 0xff;
        const int led_num = 5;
		//GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8)));
		//GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)((~led & 0x20) >> 5));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_9, (BitAction)((~led & 0x10) >> 4));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_8, (BitAction)((~led & 0x08) >> 3));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)((~led & 0x04) >> 2));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_6, (BitAction)((~led & 0x02) >> 1));
        //GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)(~led & 0x01));
        do_value.led1 = (~led & 0x20) >> 5;
        do_value.led2 = (~led & 0x10) >> 4;
        do_value.led3 = (~led & 0x08) >> 3;
        do_value.led4 = (~led & 0x04) >> 2;
        do_value.led5 = (~led & 0x02) >> 1;
        //do_value.bit.led6 = ~led & 0x01;
        //led++;
        led = led < (1 << led_num)-1 ? (led + 1) : 0;
		vTaskDelay( 1000 / portTICK_PERIOD_MS ); /* Delay 1000 ms */
	}
}
void eth_task(void *pvParameters )
{
    //int32_t ret;
    //uint8_t i;
	( void ) pvParameters;
	for(;;)
	{
        //if ((ret = loopback_tcps(SOCK_TCPS, gDATABUF, PORT_TCPS)) < 0) // TCP server loopback test
        //{
            //GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8)));
        //}
        //for(i = 0; i < MAX_HTTPSOCK; i++)	httpServer_run(i); 	// HTTP Server handler
        const int send_len = 16;
        //int i;
        uint8_t sbuf[send_len];
        memset(sbuf,0,sizeof(sbuf));
        memcpy(sbuf,&(di_value.di_filtered),sizeof(di_value.di_filtered));
        memcpy(sbuf+sizeof(di_value.di_filtered),&(do_value.all),sizeof(do_value));
        //for(i=0;i<sizeof(di_value.di_filtered);i++)
        //{
            //sbuf[i] = 
        //}
        loop_tcps(SOCK_TCPS, tcp_rx_buf, 5000, sbuf,sizeof(sbuf));
        if(0xfa == tcp_rx_buf[0] && 0x5a == tcp_rx_buf[1])
        {
            switch(tcp_rx_buf[2])
            {
                case 0x01:
                    do_value.all = *(uint32_t *)&tcp_rx_buf[3];
                    break;
                default:
                    break;
            }
            memset(tcp_rx_buf,0,sizeof(tcp_rx_buf));
            tcp_rx_buf[0] = 0;
            tcp_rx_buf[1] = 0;
        }
        //else
        //{
            //GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1));
        //}
        vTaskDelay( 1000 / portTICK_PERIOD_MS ); /* Delay 1000 ms */
    }
}

void start_di_task(UBaseType_t prio)
{
    xTaskCreate( di_task, "DI", DI_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
void start_do_task(UBaseType_t prio)
{
    xTaskCreate( do_task, "DO", DO_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}

void start_eth_task(UBaseType_t prio)
{
    xTaskCreate( eth_task, "ETH", ETH_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
void start_led_task(UBaseType_t prio)
{
    xTaskCreate( led_task, "LED", LED_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
	//xTaskCreate( led_task1, "LED1", LED_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
static void prvSetupHardware( void )
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
	{
	}

	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = 8MHz * 9 = 72 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC, ENABLE );

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	//vParTestInitialise();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
