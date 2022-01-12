#ifndef _ROBOT_CMD_H_
#define _ROBOT_CMD_H_

#include "bsp_log.h"
#include "pub_sub.h"
#include "robot_param.h"
#include "stdint.h"
// 外设
#include "BMI088.h"
#include "can_recv.h"
#include "can_send.h"

typedef enum robot_mode_e { robot_stop, robot_run } robot_mode;

// 板间通信部分
#pragma pack(1)
// gimbal output chassis input 云台->底盘数据包
typedef struct board_com_goci_data_t {
    robot_mode now_robot_mode;  // 遥控器在云台主控 包含stop模式与云台重要模块掉线
    Chassis_param_speed_target chassis_target;
} board_com_goci_data;
// gimbal input chassis output数据包
typedef struct board_com_gico_data_t {
    enum { module_lost, module_working } if_chassis_board_module_lost;  // 同步底盘是否有重要模块掉线
    imu_data chassis_imu_data;                                          // 将底盘主控的数据发到云台
    struct {
        uint16_t bullet_speed_max;   // 弹速
        uint16_t heat_limit_remain;  // 剩余热量
    } shoot_referee_data;
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
    robot_mode mode;
    Board_com board_com;
#ifdef CHASSIS_BOARD
    Publisher* chassis_cmd_puber;
    Chassis_param chassis_param;
#endif
#ifdef GIMBAL_BOARD
    Publisher* gimbal_cmd_puber;
    Subscriber* gimbal_upload_suber;
    Publisher* shoot_puber;
#endif
} Robot;

Robot *Robot_CMD_Create(void);
void Robot_CMD_Update(Robot *obj);
#endif