#ifndef _CAN_SEND_H
#define _CAN_SEND_H
#include "datatypes.h"

#pragma pack(1)
typedef struct can_send_config_t{
    uint8_t bsp_can_index;
    uint8_t data_len;
    uint16_t can_identifier;
} can_send_config;

typedef struct can_send_t {
    can_send_config config;
    uint8_t* txbuf;
    uint8_t buf_len;
} can_send;
#pragma pack()

can_send* CanSend_Create(can_send_config* config);
void CanSend_Send(can_send* obj, uint8_t* data);
#endif