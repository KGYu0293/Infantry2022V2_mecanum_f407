#ifndef _MINIPC_H
#define _MINIPC_H
#include "stdint.h"
#include "bsp_can.h"
#include "datatypes.h"

typedef struct minipc_send_t{
    float euler_deg[3];
} minipc_send;

typedef struct minipc_recv_t{
    float x,y;
} minipc_recv;

typedef struct minipc_config_t {
    uint8_t bsp_can_index;
    uint16_t device_can_id;
} minipc_config;

typedef struct minipc_t{
    minipc_recv data;
    minipc_config config;
} minipc;

void minipc_Driver_Init();
minipc* minipc_create(minipc_config* config);
#endif