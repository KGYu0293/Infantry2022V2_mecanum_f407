#ifndef _ROBOT_CMD_H_
#define _ROBOT_CMD_H_

#include "app.h"

// 板间通信部分
#pragma pack(1)
// gimbal output chassis input 云台->底盘数据包
typedef struct board_com_goci_data_t {
    uint8_t gimbal_yaw_data;
} board_com_goci_data;
// gimbal input chassis output数据包
typedef struct board_com_gico_data_t {
    uint8_t b;
} board_com_gico_data;
#pragma pack()

typedef struct board_com_t {
    can_recv *recv;
    can_send *send;
    board_com_goci_data *goci_data;
    board_com_gico_data *gico_data;
} Board_com;

// command结构体
typedef struct Robot_t {
    enum {
        robot_stop,
        robot_run
    } mode;
    Board_com board_com;
    // Chassis *chassis;
} Robot;

Robot *Robot_Create(void);
#endif