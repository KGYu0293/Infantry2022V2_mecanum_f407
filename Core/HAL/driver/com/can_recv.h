#ifndef _CAN_RECV_H
#define _CAN_RECV_H
#include "stdint.h"
#include "datatypes.h"

typedef struct can_recv_config_t{
    uint8_t bsp_can_index;
    uint8_t data_len;
    uint8_t data_id;
    uint8_t data_type;
    uint16_t can_identifier;
} can_recv_config;

typedef struct can_recv_t{
    can_recv_config config;
    general_data data_rx;
    uint8_t buf_len;
    uint8_t recv_len;
    uint8_t recv_status;
    uint8_t* rxbuf;
} can_recv;

void CanRecv_Driver_Init();
can_recv* CanRecv_Create(can_recv_config* config);
#endif