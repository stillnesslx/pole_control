//Ruixiaoliang 20201215 16:24
#include "relay.h"

#define RELAY_DPDT_DEFAULT {RELEASE,DONE,OPEN,CLOSE}

struct relays_in_fact relays = {RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT
,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT
,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT
,RELAY_DPDT_DEFAULT,RELAY_DPDT_DEFAULT};
struct relays_in_fact relays_pre;


//struct relay_DPDT lift_remote = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT drop_remote = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT stop_remote = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT usr1_remote = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT usr2_remote = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT usr3_remote = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT sensor_in1 = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT sensor_in2 = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT sensor_in3 = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT lift1 = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT lift2 = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT drop1 = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT drop2 = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT stop_local = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT motor_start = {RELEASE,OPEN,CLOSE};
//struct relay_DPDT brake_power = {RELEASE,OPEN,CLOSE};


void set_relay_coil(struct relay_DPDT *rly, RelayCoilOR ror)
{
    rly->coil = ror;
    rly->state = UNDONE;
    //rly->open_contact = RELEASE == ror ? OPEN : CLOSE;
    //rly->close_contact = RELEASE == ror ? CLOSE : OPEN;
    //rly->or_flag = TRUE;
}

void set_relay_contact(struct relay_DPDT *rly)
{
    //rly->coil = ror;
    rly->state = DONE;
    rly->open_contact = RELEASE == rly->coil ? OPEN : CLOSE;
    rly->close_contact = RELEASE == rly->coil ? CLOSE : OPEN;
    //rly->or_flag = TRUE;
}
void set_relay_contact_operate(struct relay_DPDT *rly)
{
    //rly->coil = ror;
    //rly->state = DONE;
    rly->open_contact = CLOSE;
    rly->close_contact = OPEN;
    //rly->or_flag = TRUE;
}
void set_relay_contact_release(struct relay_DPDT *rly)
{
    //rly->coil = ror;
    //rly->state = DONE;
    rly->open_contact = OPEN;
    rly->close_contact = CLOSE;
    //rly->or_flag = TRUE;
}
//end of file
