/* 演示如何基于can_send/can_recv 构建特质化的通信*/
#ifndef _MINIPC_H
#define _MINIPC_H
#include "can_recv.h"
#include "can_send.h"
#include "datatypes.h"
#include "monitor.h"
#include "stdint.h"

#pragma pack(1)
typedef struct canpc_send_t {
    float euler[3];
    uint8_t auto_mode_flag;
    uint8_t robot_id;
    uint8_t bullet_speed;
} canpc_send;

typedef struct canpc_recv_t {
    float pitch;
    float roll;
    float yaw;
    float wait_time;// 开火延迟时间
} canpc_recv;
#pragma pack()

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
    canpc_recv* pc_recv_data;
    uint8_t* data_updated;
} canpc;

canpc* CanPC_Create(canpc_config* config);
void CanPC_Send(canpc* obj, canpc_send* data);
#endif