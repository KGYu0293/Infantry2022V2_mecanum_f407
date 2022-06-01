#ifndef _CAN_RECV_H
#define _CAN_RECV_H
#include "datatypes.h"
#include "bsp_supervise.h"
#include "stdint.h"
#include "monitor.h"

struct can_recv_config_t;
struct can_recv_t;
typedef struct can_recv_t can_recv;
typedef struct can_recv_config_t can_recv_config;
typedef void (*can_recv_notifiy)(can_recv* obj);

struct can_recv_config_t {
    uint8_t bsp_can_index;
    uint8_t data_len;
    uint16_t can_identifier;
    can_recv_notifiy notify_func;
    lost_callback lost_callback;
};

struct can_recv_t {
    can_recv_config config;
    general_data data_rx;
    uint8_t buf_len;
    uint8_t recv_len;
    uint8_t recv_status;
    uint8_t data_updated;
    uint8_t* rxbuf;
    monitor_item* monitor;
    FPS_t fps;
};

void CanRecv_Driver_Init();
can_recv* CanRecv_Create(can_recv_config* config);
#endif