#ifndef _PC_DATA_H
#define _PC_DATA_H

#include <stdint.h>
#pragma pack(1)
typedef struct pc_send_t {
    float euler[3];
    uint8_t auto_mode_flag;
    uint8_t robot_id;
    uint8_t bullet_speed;
} pc_send;

typedef struct pc_recv_t {
    float pitch;
    float roll;
    float yaw;
    float wait_time;  // 开火延迟时间
} pc_recv;
#pragma pack()

#endif