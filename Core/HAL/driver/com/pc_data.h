#ifndef _PC_DATA_H
#define _PC_DATA_H

#include <stdint.h>

#define VISUAL_NO_TARGET 0    // 找不到目标
#define VISUAL_FOLLOW 1       // 跟随目标
#define VISUAL_FIRE_SINGLE 2  // 单发（打符）

#pragma pack(1)
typedef struct pc_send_t {
    float euler[3];
    uint8_t auto_mode_flag;
    uint8_t robot_id;
    float bullet_speed;
} pc_send;

typedef struct pc_recv_t {
    float roll;
    float pitch;
    float yaw;
    uint8_t vitual_mode;
} pc_recv;
#pragma pack()

#endif