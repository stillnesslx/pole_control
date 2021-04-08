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
#include "relay.h"
//#include "httpServer.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WORK_PERIOD 10
#define RELAY_CLOSE 0
#define RELAY_OPEN  1
#define RELAY_OR_TIME_MAX 100
#define RELAY_OR_TIME_OUT (RELAY_OR_TIME_MAX/WORK_PERIOD)

/* Task priorities. */
#define DI_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define DO_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define WORK_TASK_PRIORITY				( tskIDLE_PRIORITY + 4 )
#define LED_TASK_PRIORITY				( tskIDLE_PRIORITY + 6 )
#define ETH_TASK_PRIORITY				( tskIDLE_PRIORITY + 7 )
#define FAULT_TASK_PRIORITY				( tskIDLE_PRIORITY + 7 )
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
#define SOCK_TCPS       0
#define PORT_TCPS		5000
/* Private variables ---------------------------------------------------------*/
//char RxBuffer1[512];
//uint8_t RxCounter1=0;

//u8 hx = 24,hy = 0,hlen = 3,hsize = 32;
//u32 h = 60;
//u8 test_en=0;
//const uint8_t high[3] = {50,100,150};
//SemaphoreHandle_t xSemaphore = NULL;


//#define DATA_BUF_SIZE   2048 	// defined in loopback.h
//#define DATA_BUF_SIZE   64 	// defined in loopback.h
uint8_t gDATABUF[DATA_BUF_SIZE];

//#define MAX_HTTPSOCK	1

uint8_t tcp_rx_buf[DATA_BUF_SIZE];
SemaphoreHandle_t xSemaphoreDi = NULL;

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
void start_work_task(UBaseType_t prio);
void start_fault_task(UBaseType_t prio);

static void prvSetupHardware( void );
/* Private functions ---------------------------------------------------------*/
bool lift_condition(struct relays_in_fact *relays);
bool drop_condition(struct relays_in_fact *relays);
void fault_check(uint32_t timer);

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
    
    vSemaphoreCreateBinary( xSemaphoreDi );
    //xSemaphoreTake( xSemaphoreDi, portMAX_DELAY );
    delay_ms(200);
    bsp_init();
    read_di(&di_value);
    di_value_pre.all = di_value.di_filtered;
    start_do_task(DO_TASK_PRIORITY);
    start_di_task(DI_TASK_PRIORITY);
    start_work_task(WORK_TASK_PRIORITY);
    start_led_task(LED_TASK_PRIORITY);
    start_fault_task(FAULT_TASK_PRIORITY);
    start_eth_task(ETH_TASK_PRIORITY);
    if(NULL != xSemaphoreDi)
    {
        start_work_task(WORK_TASK_PRIORITY);
    }
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
#define WORK_STACK_SIZE	            ( ( unsigned short ) 64 )
#define FAULT_STACK_SIZE	        ( ( unsigned short ) 32 )
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

static void fault_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        static uint32_t timer = 0;
        fault_check(timer);
        ++timer;
        
		vTaskDelay( 100 / portTICK_PERIOD_MS ); /* Delay 100 ms */
	}
}

//static struct di_data di_value_pre;
static void work_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        union di_define di_cflg;
        bool flag = FALSE;
        //static uint32_t timer = 0;
        //fault_check(timer);
        //++timer;
        xSemaphoreTake( xSemaphoreDi, portMAX_DELAY );

        di_cflg.all = 0;
        di_cflg.all = di_value.di_filtered ^ di_value_pre.all;
        di_value_pre.all = di_value.di_filtered;
        //feed back relay. read feedback di and set contact//////////////////////////////////////////////////////////////////
        if(TRUE == di_cflg.bit.brake)// && UNDONE == relays.brake_power.state)
        {
//            if((DI_TRUE == di_value.bit.brake && OPERATE == relays.brake_power.coil)
//                || (DI_FALSE == di_value.bit.brake && RELEASE == relays.brake_power.coil))
//            {
//                set_relay_contact(&relays.brake_power);
//            }
//            else
//            {
//                fault_code.bit.relay_state = TRUE;
//            }
            if(DI_TRUE == di_value.bit.brake)
            {
                set_relay_contact_operate(&relays.brake_power);
            }
            else
            {
                set_relay_contact_release(&relays.brake_power);
            }
        }
        if(TRUE == di_cflg.bit.z1)// && UNDONE == relays.motor_start.state)
        {
//            if((DI_TRUE == di_value.bit.z1 && OPERATE == relays.motor_start.coil)
//                || (DI_FALSE == di_value.bit.z1 && RELEASE == relays.motor_start.coil))
//            {
//                set_relay_contact(&relays.motor_start);
//            }
//            else
//            {
//                fault_code.bit.relay_state = TRUE;
//            }
            if(DI_TRUE == di_value.bit.z1)
            {
                set_relay_contact_operate(&relays.motor_start);
            }
            else
            {
                set_relay_contact_release(&relays.motor_start);
            }

        }
        if(TRUE == di_cflg.bit.usr1o)// && UNDONE == relays.usr1_remote.state)
        {
//            if((DI_TRUE == di_value.bit.usr1o && OPERATE == relays.usr1_remote.coil)
//                || (DI_FALSE == di_value.bit.usr1o && RELEASE == relays.usr1_remote.coil))
//            {
//                set_relay_contact(&relays.usr1_remote);
//            }
//            else
//            {
//                fault_code.bit.relay_state = TRUE;
//            }
            if(DI_TRUE == di_value.bit.usr1o)
            {
                set_relay_contact_operate(&relays.usr1_remote);
            }
            else
            {
                set_relay_contact_release(&relays.usr1_remote);
            }
        }
        if(TRUE == di_cflg.bit.usr2o)// && UNDONE == relays.usr2_remote.state)
        {
//            if((DI_TRUE == di_value.bit.usr2o && OPERATE == relays.usr2_remote.coil)
//                || (DI_FALSE == di_value.bit.usr2o && RELEASE == relays.usr2_remote.coil))
//            {
//                set_relay_contact(&relays.usr2_remote);
//            }
//            else
//            {
//                fault_code.bit.relay_state = TRUE;
//            }
            if(DI_TRUE == di_value.bit.usr2o)
            {
                set_relay_contact_operate(&relays.usr2_remote);
            }
            else
            {
                set_relay_contact_release(&relays.usr2_remote);
            }
        }
        if(TRUE == di_cflg.bit.usr3o)// && UNDONE == relays.usr3_remote.state)
        {
//            if((DI_TRUE == di_value.bit.usr3o && OPERATE == relays.usr3_remote.coil)
//                || (DI_FALSE == di_value.bit.usr3o && RELEASE == relays.usr3_remote.coil))
//            {
//                set_relay_contact(&relays.usr3_remote);
//            }
//            else
//            {
//                fault_code.bit.relay_state = TRUE;
//            }
            if(DI_TRUE == di_value.bit.usr3o)
            {
                set_relay_contact_release(&relays.usr3_remote);
            }
            else
            {
                set_relay_contact_operate(&relays.usr3_remote);
            }
        }
        //sensor drive relay directly.//////////////////////////////////////////////////////////////////
        if(TRUE == di_cflg.bit.sen1)
        {
            if(DI_TRUE == di_value.bit.sen1)
            {
                set_relay_coil(&relays.sensor_in1,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.sensor_in1,RELEASE);
            }
            set_relay_contact(&relays.sensor_in1);
        }
        if(TRUE == di_cflg.bit.sen2)
        {
            if(DI_TRUE == di_value.bit.sen2)
            {
                set_relay_coil(&relays.sensor_in2,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.sensor_in2,RELEASE);
            }
            set_relay_contact(&relays.sensor_in2);
        }
        if(TRUE == di_cflg.bit.sen3)
        {
            if(DI_TRUE == di_value.bit.sen3)
            {
                set_relay_coil(&relays.sensor_in3,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.sensor_in3,RELEASE);
            }
            set_relay_contact(&relays.sensor_in3);
        }

        //virtual relay. Set contact immediately.//////////////////////////////////////////////////////////////////
        if(TRUE == di_cflg.bit.liftr || TRUE == di_cflg.bit.liftl || TRUE == di_cflg.bit.key1)
        {
            if(DI_TRUE == di_value.bit.liftr || DI_TRUE == di_value.bit.liftl || DI_TRUE == di_value.bit.key1)
            {
                set_relay_coil(&relays.lift_remote,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.lift_remote,RELEASE);
            }
            set_relay_contact(&relays.lift_remote);
        }
        if(TRUE == di_cflg.bit.dropr || TRUE == di_cflg.bit.dropl || TRUE == di_cflg.bit.key2)
        {
            if(DI_TRUE == di_value.bit.dropr || DI_TRUE == di_value.bit.dropl || DI_TRUE == di_value.bit.key2)
            {
                set_relay_coil(&relays.drop_remote,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.drop_remote,RELEASE);
            }
            set_relay_contact(&relays.drop_remote);
        }
        if(TRUE == di_cflg.bit.stopr)
        {
            if(DI_TRUE == di_value.bit.stopr)
            {
                set_relay_coil(&relays.stop_remote,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.stop_remote,RELEASE);
            }
            set_relay_contact(&relays.stop_remote);
        }
        //usr1,2,3 remote and local relay//////////////////////////////////////////////////////////////////
        if(TRUE == di_cflg.bit.usr1r)
        {
            if(DI_TRUE == di_value.bit.usr1r || DI_TRUE == di_value.bit.usr1l)
            {
                set_relay_coil(&relays.usr1_remote,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.usr1_remote,RELEASE);
            }
            //set_relay_contact(&relays.usr1_remote);
        }
        if(TRUE == di_cflg.bit.usr2r)
        {
            if(DI_TRUE == di_value.bit.usr2r || DI_TRUE == di_value.bit.usr2l)
            {
                set_relay_coil(&relays.usr2_remote,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.usr2_remote,RELEASE);
            }
            //set_relay_contact(&relays.usr2_remote);
        }
        if(TRUE == di_cflg.bit.usr3r || TRUE == di_cflg.bit.usr4r)
        {
            //if(DI_TRUE == di_value.bit.usr3r || DI_TRUE == di_value.bit.usr4r || DI_TRUE == di_value.bit.key1 || DI_TRUE == di_value.bit.key2)
            if(DI_TRUE == di_value.bit.usr3r || DI_TRUE == di_value.bit.usr4r)
            {
                set_relay_coil(&relays.usr3_remote,OPERATE);
            }
            else
            {
                set_relay_coil(&relays.usr3_remote,RELEASE);
            }
            //set_relay_contact(&relays.usr3_remote);
        }
        //local stop relay
        if(TRUE == di_cflg.bit.stopl || TRUE == di_cflg.bit.stop_sw3)
        {
            if(RELEASE == relays.stop_local.coil && (DI_TRUE == di_value.bit.stopl || DI_TRUE == di_value.bit.stop_sw3))
            {
                set_relay_coil(&relays.stop_local,OPERATE);
            }
            else if(OPERATE == relays.stop_local.coil && (DI_FALSE == di_value.bit.stopl && DI_FALSE == di_value.bit.stop_sw3))
            {
                set_relay_coil(&relays.stop_local,RELEASE);
            }
            else
            {
                //wait action
            }
            set_relay_contact(&relays.stop_local);
        }
        //if(TRUE == di_cflg.bit.liftl)
        //{
            
        //}

        //virtual relay. Set contact immediately.//////////////////////////////////////////////////////////////////
        //lift relay
        flag = lift_condition(&relays);
        //if(RELEASE == relays.lift.coil// && RELEASE == relays.lift1.coil
        if(RELEASE == relays.lift.coil && TRUE == flag
            //&& (CLOSE == relays.lift_remote.open_contact || CLOSE == relays.lift1.open_contact || DI_TRUE == di_value.bit.lift_sw1 || CLOSE == di_value.bit.liftl)
            //&& CLOSE == relays.drop_remote.close_contact
            //&& CLOSE == relays.stop_local.close_contact
            //&& CLOSE == relays.sensor_in1.close_contact
            )
        {
            set_relay_coil(&relays.lift,OPERATE);
            set_relay_contact(&relays.lift);
            //set_relay_contact(&relays.lift2,OPERATE);
        }
        //else if(OPERATE == relays.lift.coil && OPERATE == relays.lift1.coil)
        else if(OPERATE == relays.lift.coil && FALSE == flag)
        {
            set_relay_coil(&relays.lift,RELEASE);
            set_relay_contact(&relays.lift);
            //set_relay_contact(&relays.lift2,RELEASE);
        }
        else
        {
            //fault_code.bit.relay_or_notfinish = TRUE;
        }
        //drop relay
        flag = drop_condition(&relays);
        //if(RELEASE == relays.drop.coil && RELEASE == relays.drop1.coil && TRUE == flag
        if(RELEASE == relays.drop.coil && TRUE == flag
            //&& (CLOSE == relays.drop_remote.open_contact || CLOSE == relays.drop1.open_contact || DI_TRUE == di_value.bit.drop_sw2 || CLOSE == di_value.bit.dropl)
            //&& CLOSE == relays.lift_remote.close_contact
            //&& CLOSE == relays.stop_local.close_contact
            //&& CLOSE == relays.sensor_in2.close_contact
            //&& CLOSE == relays.sensor_in3.open_contact
            )
        {
            set_relay_coil(&relays.drop,OPERATE);
            set_relay_contact(&relays.drop);
            //set_relay_contact(&relays.drop2,OPERATE);
        }
        //else if(OPERATE == relays.drop1.coil && OPERATE == relays.drop2.coil && FALSE == flag)
        else if(OPERATE == relays.drop.coil && FALSE == flag)
        {
            set_relay_coil(&relays.drop,RELEASE);
            set_relay_contact(&relays.drop);
            //set_relay_contact(&relays.drop2,RELEASE);
        }
        else
        {
            //if(relays.drop1.coil != relays.drop2.coil)
            //{
                //fault_code.bit.lift12_neq = TRUE;
            //}
            //fault_code.bit.relay_or_notfinish = TRUE;
            //wait action
        }
        //brake power relay
        if(RELEASE == relays.brake_power.coil
            && ((CLOSE == relays.lift.open_contact && OPEN == relays.drop.open_contact)
                || (OPEN == relays.lift.open_contact && CLOSE == relays.drop.open_contact)))
        {
            set_relay_coil(&relays.brake_power,OPERATE);
        }
        else if(OPERATE == relays.brake_power.coil
            && (OPEN == relays.lift.open_contact && OPEN == relays.drop.open_contact))
        {
            set_relay_coil(&relays.brake_power,RELEASE);
        }
        else
        {
            if(CLOSE == relays.lift.open_contact && CLOSE == relays.drop.open_contact)
            {
                //lift drop conflict
            }
        }
        //motor start relay
        if(RELEASE == relays.motor_start.coil
            && ((CLOSE == relays.lift.open_contact && OPEN == relays.drop.open_contact)
                || (OPEN == relays.lift.open_contact && CLOSE == relays.drop.open_contact))
            && CLOSE == relays.brake_power.open_contact)
        {
            set_relay_coil(&relays.motor_start,OPERATE);
        }
        else if(OPERATE == relays.motor_start.coil
            && ((OPEN == relays.lift.open_contact && OPEN == relays.drop.open_contact)
            || OPEN == relays.brake_power.open_contact))
        {
            set_relay_coil(&relays.motor_start,RELEASE);
        }
        else
        {
            if(CLOSE == relays.lift.open_contact && CLOSE == relays.drop.open_contact)
            {
                //lift drop conflict
            }
        }

        //do_value.m_brake = CLOSE == relays.brake_power.open_contact ? DO_VALID : DO_INVALID;
        //do_value.m_start = CLOSE == relays.motor_start.open_contact ?  DO_VALID : DO_INVALID;
        do_value.m_brake = OPERATE == relays.brake_power.coil ? DO_VALID : DO_INVALID;
        do_value.m_start = OPERATE == relays.motor_start.coil ?  DO_VALID : DO_INVALID;
        //do_value.lift_drop = CLOSE == relays.motor_start.open_contact && CLOSE == relays.lift.open_contact && OPEN == relays.drop.open_contact ? DO_VALID : DO_INVALID; //drop2
        //if(CLOSE == relays.motor_start.open_contact 
        if(CLOSE == relays.lift.open_contact && OPEN == relays.drop.open_contact)
        {
            do_value.lift_drop = DO_VALID;
        }
        //else if(CLOSE == relays.motor_start.open_contact 
        else if(OPEN == relays.lift.open_contact && CLOSE == relays.drop.open_contact)
        {
            do_value.lift_drop = DO_INVALID;
        }
        else
        {
            
        }
        do_value.usr1 = CLOSE == relays.usr1_remote.open_contact ? DO_VALID : DO_INVALID;
        do_value.usr2 = CLOSE == relays.usr2_remote.open_contact ? DO_VALID : DO_INVALID;
        do_value.usr3 = CLOSE == relays.usr3_remote.open_contact ? DO_VALID : DO_INVALID;

        //do_value.m_brake = DO_INVALID;
        //do_value.m_start = DO_INVALID;
        //do_value.lift_drop = DO_VALID; //drop2
        //do_value.lift_drop = DI_TRUE == di_value.bit.sw4 ? DO_VALID : DO_INVALID;
        //do_value.usr1 = DO_INVALID;
        //do_value.usr2 = DO_INVALID;
        //do_value.usr3 = DO_INVALID;
        
		vTaskDelay( WORK_PERIOD / portTICK_PERIOD_MS ); /* Delay 10 ms */
	}
}
static void di_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        read_di(&di_value);
        if(di_value_pre.all != di_value.di_filtered)
        {
            xSemaphoreGive( xSemaphoreDi );
        }
		vTaskDelay( 100 / portTICK_PERIOD_MS ); /* Delay 100 ms */
	}
}
static void do_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        //read_di(&di_value);
        write_do(do_value);
		vTaskDelay( 200 / portTICK_PERIOD_MS ); /* Delay 200     ms */
	}
}
static void led_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
        //static uint8_t led = 0xff;
        //const int led_num = 5;
		//GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8)));
		//GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)((~led & 0x20) >> 5));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_9, (BitAction)((~led & 0x10) >> 4));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_8, (BitAction)((~led & 0x08) >> 3));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)((~led & 0x04) >> 2));
        //GPIO_WriteBit(GPIOC, GPIO_Pin_6, (BitAction)((~led & 0x02) >> 1));
        //GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)(~led & 0x01));
        //do_value.led1 = (~led & 0x20) >> 5;
        //do_value.led2 = (~led & 0x10) >> 4;
        //do_value.led3 = (~led & 0x08) >> 3;
        //do_value.led4 = (~led & 0x04) >> 2;
        //do_value.led5 = (~led & 0x02) >> 1;
        ////do_value.bit.led6 = ~led & 0x01;
        ////led++;
        //led = led < (1 << led_num)-1 ? (led + 1) : 0;
//        do_value.led1 = fault_code.bit.brake_relay_timeout;
//        do_value.led2 = fault_code.bit.lift_drop_conflict;
//        do_value.led3 = fault_code.bit.start_relay_timeout;
//        do_value.led4 = fault_code.bit.usr1_relay_timeout;
//        do_value.led5 = fault_code.bit.usr2_relay_timeout;
//        do_value.led6 = fault_code.bit.usr3_relay_timeout;
        do_value.led1 = do_value.lift_drop;
        do_value.led2 = do_value.m_start;
        do_value.led3 = do_value.m_brake;
        do_value.led4 = do_value.usr1;
        do_value.led5 = do_value.usr2;
        do_value.led6 = do_value.usr3;

        ////do_value.bit.led6 = ~led & 0x01;

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
        const int send_len = 32;
        //int i;
        uint8_t sbuf[send_len];
        memset(sbuf,0,sizeof(sbuf));
        memcpy(sbuf,&(di_value.di_filtered),sizeof(di_value.di_filtered));
        memcpy(sbuf+sizeof(di_value.di_filtered),&(do_value.all),sizeof(do_value));
        memcpy(sbuf+sizeof(di_value.di_filtered)+sizeof(do_value),&(fault_code.all),sizeof(fault_code.all));
        memcpy(sbuf+sizeof(di_value.di_filtered)+sizeof(do_value)+sizeof(fault_code.all),&(relays),sizeof(relays));
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
                    //do_value.all = *(uint32_t *)&tcp_rx_buf[3];
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

void start_work_task(UBaseType_t prio)
{
    xTaskCreate( work_task, "WORK", WORK_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
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
void start_fault_task(UBaseType_t prio)
{
    xTaskCreate( fault_task, "WORK", FAULT_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}


bool lift_condition(struct relays_in_fact *relays)
{
    //if((CLOSE == relays->lift_remote.open_contact || CLOSE == relays->lift1.open_contact || DI_TRUE == di_value.bit.lift_sw1 || CLOSE == di_value.bit.liftl)
    if((CLOSE == relays->lift_remote.open_contact || CLOSE == relays->lift.open_contact)
            && CLOSE == relays->drop_remote.close_contact
            && CLOSE == relays->stop_remote.close_contact
            && CLOSE == relays->stop_local.close_contact
            && CLOSE == relays->sensor_in1.close_contact)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
bool drop_condition(struct relays_in_fact *relays)
{
    //if((CLOSE == relays->drop_remote.open_contact || CLOSE == relays->drop1.open_contact || DI_TRUE == di_value.bit.drop_sw2 || CLOSE == di_value.bit.dropl)
    if((CLOSE == relays->drop_remote.open_contact || CLOSE == relays->drop.open_contact)
        && CLOSE == relays->lift_remote.close_contact
        && CLOSE == relays->stop_local.close_contact
        && CLOSE == relays->stop_remote.close_contact
        && CLOSE == relays->sensor_in2.close_contact
        && CLOSE == relays->sensor_in3.open_contact)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
void fault_check(uint32_t timer)
{
    static uint32_t brake_timer = 0;
    static uint32_t start_timer = 0;
    static uint32_t usr1_timer = 0;
    static uint32_t usr2_timer = 0;
    static uint32_t usr3_timer = 0;
    if((DI_TRUE == di_value.bit.lift_sw1 || DI_TRUE == di_value.bit.liftr || DI_TRUE == di_value.bit.liftl)
        && (DI_TRUE == di_value.bit.drop_sw2 || DI_TRUE == di_value.bit.dropr || DI_TRUE == di_value.bit.dropl))
    {
        fault_code.bit.lift_drop_conflict = FAULT;
    }
    else
    {
        fault_code.bit.lift_drop_conflict = NORMAL;
    }
    //if((DO_VALID == do_value.m_brake && DI_TRUE == di_value.bit.brake)
    if((OPERATE == relays.brake_power.coil && DI_TRUE == di_value.bit.brake)
        || (RELEASE == relays.brake_power.coil && DI_FALSE == di_value.bit.brake))
    {
        brake_timer = 0;
        fault_code.bit.brake_relay_timeout = NORMAL;
    }
    else
    {
        ++brake_timer;
        if(brake_timer > RELAY_OR_TIME_OUT)
        {
            fault_code.bit.brake_relay_timeout = FAULT;
        }
    }
    if((OPERATE == relays.motor_start.coil && DI_TRUE == di_value.bit.z1)
        || (RELEASE == relays.motor_start.coil && DI_FALSE == di_value.bit.z1))
    {
        start_timer = 0;
        fault_code.bit.start_relay_timeout = NORMAL;
    }
    else
    {
        ++start_timer;
        if(start_timer > RELAY_OR_TIME_OUT)
        {
            fault_code.bit.start_relay_timeout = FAULT;
        }
    }
    
    if((OPERATE == relays.usr1_remote.coil && DI_TRUE == di_value.bit.usr1o)
        || (RELEASE == relays.usr1_remote.coil && DI_FALSE == di_value.bit.usr1o))
    {
        usr1_timer = 0;
        fault_code.bit.usr1_relay_timeout = NORMAL;
    }
    else
    {
        ++usr1_timer;
        if(usr1_timer > RELAY_OR_TIME_OUT)
        {
            fault_code.bit.usr1_relay_timeout = FAULT;
        }
    }
    
    if((OPERATE == relays.usr2_remote.coil && DI_TRUE == di_value.bit.usr2o)
        || (RELEASE == relays.usr2_remote.coil && DI_FALSE == di_value.bit.usr2o))
    {
        usr2_timer = 0;
        fault_code.bit.usr2_relay_timeout = NORMAL;
    }
    else
    {
        ++usr2_timer;
        if(usr2_timer > RELAY_OR_TIME_OUT)
        {
            fault_code.bit.usr2_relay_timeout = FAULT;
        }
    }
    
    if((OPERATE == relays.usr3_remote.coil && DI_TRUE == di_value.bit.usr3o)
        || (RELEASE == relays.usr3_remote.coil && DI_FALSE == di_value.bit.usr3o))
    {
        usr3_timer = 0;
        fault_code.bit.usr3_relay_timeout = NORMAL;
    }
    else
    {
        ++usr3_timer;
        if(usr3_timer > RELAY_OR_TIME_OUT)
        {
            fault_code.bit.usr3_relay_timeout = FAULT;
        }
    }

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
