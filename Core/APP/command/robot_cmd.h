#ifndef _ROBOT_CMD_H_
#define _ROBOT_CMD_H_

#include "bsp_log.h"
#include "pub_sub.h"
#include "robot_param.h"
#include "stdint.h"
// 外设
#include "BMI088.h"
#include "DT7_DR16.h"
#include "can_recv.h"
#include "can_send.h"
#include "referee.h"

typedef enum robot_mode_e { robot_stop, robot_run } robot_mode;

// 板间通信部分
#pragma pack(1)
// gimbal output chassis input 云台->底盘数据包
typedef struct board_com_goci_data_t {
    robot_mode now_robot_mode;  // 遥控器在云台主控 包含stop模式与云台重要模块掉线
    Chassis_mode chassis_mode;
    Chassis_param_speed_target chassis_target;
} board_com_goci_data;
// gimbal input chassis output数据包
typedef struct board_com_gico_data_t {
    Module_status chassis_board_module_status;  // 同步底盘是否有重要模块掉线
    float gyro_yaw;                            // 将底盘主控的imu数据发到云台
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
#ifdef GIMBAL_BOARD
typedef struct Robot_t {
    Board_com board_com;
    robot_mode mode;
    Module_status if_gimbal_imu_lost;

    dt7Remote *remote;

    Publisher *gimbal_cmd_puber;
    Gimbal_param gimbal_param;
    Publisher *shoot_cmd_puber;
    Shoot_param shoot_param;
    Subscriber *gimbal_upload_suber;
} Robot;
#endif
#ifdef CHASSIS_BOARD
typedef struct Robot_t {
    Board_com board_com;
    robot_mode mode;

    Referee *referee;

    Publisher *chassis_cmd_puber;
    Chassis_param chassis_param;  // 将要pub的变量定义在结构体中以长期保存(pub的是指针，要放在指针不会销毁的地方)
    Subscriber *chassis_upload_sub;
} Robot;
#endif

Robot *Robot_CMD_Create(void);
void Robot_CMD_Update(Robot *obj);
#endif