/**
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "fifo_buffer.h"

#pragma anon_unions
     
typedef enum {FALSE = 0, TRUE = 1} bool;
typedef enum {NORMAL = 0, FAULT = 1} FAULT_STATE;
//DI PORTA PINs
#define DI_PIN_USR2R        GPIO_Pin_0
#define DI_PIN_USR3R        GPIO_Pin_1
#define DI_PIN_USR4R        GPIO_Pin_2
#define DI_PIN_Z1           GPIO_Pin_11
#define DI_PIN_BRAKE        GPIO_Pin_15
//PORTB PINs
#define DI_PIN_SW8          GPIO_Pin_0
#define DI_PIN_SW7          GPIO_Pin_1
#define DI_PIN_SW6          GPIO_Pin_2
#define DI_PIN_USR2O        GPIO_Pin_4
#define DI_PIN_SEN1         GPIO_Pin_5
#define DI_PIN_SEN2         GPIO_Pin_6
#define DI_PIN_SEN3         GPIO_Pin_7
#define DI_PIN_LIFTR        GPIO_Pin_8
#define DI_PIN_DROPR        GPIO_Pin_9
#define DI_PIN_SW5          GPIO_Pin_10
#define DI_PIN_SW4          GPIO_Pin_11
#define DI_PIN_STOP_SW3     GPIO_Pin_12
#define DI_PIN_DROP_SW2     GPIO_Pin_13
#define DI_PIN_LIFT_SW1     GPIO_Pin_14
//PORTC PINs
#define DI_PIN_LIFTL        GPIO_Pin_0
#define DI_PIN_DROPL        GPIO_Pin_1
#define DI_PIN_STOPL        GPIO_Pin_2
#define DI_PIN_USR1R        GPIO_Pin_3
#define DI_PIN_KEY1         GPIO_Pin_4
#define DI_PIN_KEY2         GPIO_Pin_5
#define DI_PIN_USR3O        GPIO_Pin_11
#define DI_PIN_STOPR        GPIO_Pin_13
#define DI_PIN_USR1L        GPIO_Pin_14
#define DI_PIN_USR2L        GPIO_Pin_15
//PORTD PINs
#define DI_PIN_USR1O        GPIO_Pin_2

//DI PORTA PINs
#define DI_NUM_USR2R        0
#define DI_NUM_USR3R        1
#define DI_NUM_USR4R        2
#define DI_NUM_Z1           3
#define DI_NUM_BRAKE        4
//PORTB PINs
#define DI_NUM_SW8          5
#define DI_NUM_SW7          6
#define DI_NUM_SW6          7
#define DI_NUM_USR2O        8
#define DI_NUM_SEN1         9
#define DI_NUM_SEN2         10
#define DI_NUM_SEN3         11
#define DI_NUM_LIFTR        12
#define DI_NUM_DROPR        13
#define DI_NUM_SW5          14
#define DI_NUM_SW4          15
#define DI_NUM_STOP_SW3     16
#define DI_NUM_DROP_SW2     17
#define DI_NUM_LIFT_SW1     18
//PORTC PINs
#define DI_NUM_LIFTL        19
#define DI_NUM_DROPL        20
#define DI_NUM_STOPL        21
#define DI_NUM_USR1R        22
#define DI_NUM_KEY1         23
#define DI_NUM_KEY2         24
#define DI_NUM_USR3O        25
#define DI_NUM_STOPR        26
#define DI_NUM_USR1L        27
#define DI_NUM_USR2L        28
//PORTD PINs
#define DI_NUM_USR1O        29

#define DI_CHANNEL_NUM 30
#define DI_FILTER_VALUE 3

#define DO_CHANNEL_NUM 12

#define DI_INIT_VALUE 0x3fffffff

#define SWAP16(s) ((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))
#define SWAP32(l) (((l) >> 24) | (((l) & 0x00ff0000) >> 8) | (((l) & 0x0000ff00) << 8)  | ((l) << 24))

#define TEST_BIT(value,num) (0 != ((value) & ((uint32_t)0x01 << (num))))

#define DI_TRUE 0
#define DI_FALSE 1
#define DO_VALID 0
#define DO_INVALID 1


struct di_bit_define
{
    uint32_t usr2r : 1;
    uint32_t usr3r : 1;
    uint32_t usr4r : 1;
    uint32_t z1 : 1;
    uint32_t brake : 1;
    uint32_t sw8 : 1;
    uint32_t sw7 : 1;
    uint32_t sw6 : 1;
    uint32_t usr2o : 1;
    uint32_t sen1 : 1;
    uint32_t sen2 : 1;
    uint32_t sen3 : 1;
    uint32_t liftr : 1;
    uint32_t dropr : 1;
    uint32_t sw5 : 1;
    uint32_t sw4 : 1;
    uint32_t stop_sw3 : 1;
    uint32_t drop_sw2 : 1;
    uint32_t lift_sw1 : 1;
    uint32_t liftl : 1;
    uint32_t dropl : 1;
    uint32_t stopl : 1;
    uint32_t usr1r : 1;
    uint32_t key1 : 1;
    uint32_t key2 : 1;
    uint32_t usr3o : 1;
    uint32_t stopr : 1;
    uint32_t usr1l : 1;
    uint32_t usr2l : 1;
    uint32_t usr1o : 1;
    uint32_t rsv : 2;
};
union di_define
{
    uint32_t all;
    struct di_bit_define bit;
};
struct di_data
{
    uint32_t di_new;
    //u32 di_old;
    union
    {
        uint32_t di_filtered;
        struct di_bit_define bit;
        /*struct
        {
            uint32_t usr2r : 1;
            uint32_t usr3r : 1;
            uint32_t usr4r : 1;
            uint32_t z1 : 1;
            uint32_t brake : 1;
            uint32_t sw8 : 1;
            uint32_t sw7 : 1;
            uint32_t sw6 : 1;
            uint32_t usr2o : 1;
            uint32_t sen1 : 1;
            uint32_t sen2 : 1;
            uint32_t sen3 : 1;
            uint32_t liftr : 1;
            uint32_t dropr : 1;
            uint32_t sw5 : 1;
            uint32_t sw4 : 1;
            uint32_t stop_sw3 : 1;
            uint32_t drop_sw2 : 1;
            uint32_t lift_sw1 : 1;
            uint32_t liftl : 1;
            uint32_t dropl : 1;
            uint32_t stopl : 1;
            uint32_t usr1r : 1;
            uint32_t key1 : 1;
            uint32_t key2 : 1;
            uint32_t usr3o : 1;
            uint32_t stopr : 1;
            uint32_t usr1l : 1;
            uint32_t usr2l : 1;
            uint32_t usr1o : 1;
            uint32_t rsv : 2;
        };*/
    };
    uint16_t di_timer[DI_CHANNEL_NUM];
    uint16_t di_filter_num[DI_CHANNEL_NUM];
};
struct do_data
{
    union
    {
        uint32_t all;
        struct
        {
            uint32_t led1:1;
            uint32_t lift_drop:1;
            uint32_t m_start:1;
            uint32_t m_brake:1;
            uint32_t usr2:1;
            uint32_t led6:1;
            uint32_t led5:1;
            uint32_t led4:1;
            uint32_t led3:1;
            uint32_t led2:1;
            uint32_t usr3:1;
            uint32_t usr1:1;
            uint32_t resv:20;
        };
    };
};
union fault_code_def
{
    uint32_t all;
    struct
    {
        uint32_t lift_drop_conflict : 1;
        uint32_t relay_state : 1;
        uint32_t brake_relay_timeout : 1;
        uint32_t start_relay_timeout : 1;
        uint32_t usr1_relay_timeout : 1;
        uint32_t usr2_relay_timeout : 1;
        uint32_t usr3_relay_timeout : 1;
        //uint32_t relay_or_notfinish : 1;
    }bit;
};
extern union fault_code_def fault_code;
extern struct di_data di_value;
extern union di_define di_value_pre;
//extern struct di_data di_value_pre;

extern struct display_data dsp_data;

extern void read_di(struct di_data *p);
extern void bsp_init(void);
extern void delay_ms(uint16_t t);

extern struct do_data do_value;
extern void write_do(struct do_data value);

//extern __IO uint16_t ADCConvertedValue;
//extern struct di_data di_value;
//extern struct com_send_data com_sdata;
//extern t_fifo_buffer com_fifo;
//extern uint8_t com_rev_buf[COM_REV_BUF_LEN];

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
