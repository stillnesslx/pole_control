//Ruixiaoliang 20201215 16:44

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//typedef unsigned short int bool;
//typedef enum {FALSE = 0, TRUE = 1} bool;
//typedef enum {FALSE = 0, TRUE = 1} RelayOR;
typedef enum {RELEASE = 0, OPERATE = 1} RelayCoilOR;
typedef enum {OPEN = 0, CLOSE = 1} RelayContactOC;
typedef enum {UNDONE = 0, DONE = 1} RelayORState;

#define GET_OPEN_CONTACT_STATE(coil_state) (RELEASE == (coil_state) ? OPEN : CLOSE)
#define GET_CLOSE_CONTACT_STATE(coil_state) (RELEASE == (coil_state) ? CLOSE : OPEN)

struct relay_DPDT
{
    //RelayCoilOR coil;
    //RelayORState state;
    //RelayContactOC open_contact;
    //RelayContactOC close_contact;
    uint8_t coil : 1;
    uint8_t state : 1;
    uint8_t open_contact : 1;
    uint8_t close_contact : 1;
    //RelayOR or_flag;  //set this var if relay have new action
};

struct relays_in_fact
{
    struct relay_DPDT lift_remote;
    struct relay_DPDT drop_remote;
    struct relay_DPDT stop_remote;
    struct relay_DPDT usr1_remote;
    struct relay_DPDT usr2_remote;
    struct relay_DPDT usr3_remote;
    struct relay_DPDT sensor_in1;
    struct relay_DPDT sensor_in2;
    struct relay_DPDT sensor_in3;
    struct relay_DPDT lift;
    //struct relay_DPDT lift2;
    struct relay_DPDT drop;
    //struct relay_DPDT drop2;
    struct relay_DPDT stop_local;
    struct relay_DPDT motor_start;
    struct relay_DPDT brake_power;
};
extern struct relays_in_fact relays;
extern void set_relay_coil(struct relay_DPDT *rly, RelayCoilOR ror);
extern void set_relay_contact(struct relay_DPDT *rly);
//extern RelayContactOC get_contact_state();
//enum RelayContactOC get_open_contact_state(enum RelayOR relay_or);

//extern struct relay_DPDT lift_remote;
//extern struct relay_DPDT drop_remote;
//extern struct relay_DPDT stop_remote;
//extern struct relay_DPDT usr1_remote;
//extern struct relay_DPDT usr2_remote;
//extern struct relay_DPDT usr3_remote;
//extern struct relay_DPDT sensor_in1;
//extern struct relay_DPDT sensor_in2;
//extern struct relay_DPDT sensor_in3;
//extern struct relay_DPDT lift1;
//extern struct relay_DPDT lift2;
//extern struct relay_DPDT drop1;
//extern struct relay_DPDT drop2;
//extern struct relay_DPDT stop_local;
//extern struct relay_DPDT motor_start;
//extern struct relay_DPDT brake_power;

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */
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
