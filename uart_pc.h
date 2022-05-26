#ifndef _UART_PC_H
#define _UART_PC_H
#include <pc_data.h>

typedef struct canpc_config_t {
    uint8_t bsp_can_index;
    uint16_t send_identifer;
    uint16_t recv_identifer;
    lost_callback lost_callback;
} canpc_config;

typedef struct canpc_t {
    canpc_config config;
    can_send* send;
    can_recv* recv;
    pc_recv* pc_recv_data;
    uint8_t* data_updated;
} canpc;

canpc* CanPC_Create(canpc_config* config);
void CanPC_Send(canpc* obj, pc_send* data);

#endif