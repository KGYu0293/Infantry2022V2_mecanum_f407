#ifndef _CHASSIS_H
#define _CHASSIS_H
/*
 * 本文件中，无论参照系，所有vx正方向为右，y正方向为前，旋转速度正方向为顺时针
 *
 *                 |  y正方向（底盘正前）
 *                 |
 *                 |
 *------------------------------------- x正方向（右）
 *                 |
 *                 |
 *                 |
 *                 |
 *
 */
#include "BMI088.h"
#include "bsp_log.h"
#include "can_motor.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
#include "super_cap_wuli.h"

typedef struct Powcrtl_t {
    int speed_max[3];              //限制三轴速度档位
    int speed_limit;               //功率环输出的速度限制
    struct PID_t motorpower_pid;   //功率环pid
    struct PID_t powerbuffer_pid;  //功率环pid
    float power_limit_set;         //用户设置底盘功率档位
    float power_buffer_target;     //缓冲能量目标值
} Powcrtl;

typedef struct Chassis_t {
    BMI088_imu *imu;
    // uint8_t if_supercap;  //是否具有超级电容
    // Super_cap_wuli *super_cap;
    can_motor *lf;
    can_motor *rf;
    can_motor *lb;
    can_motor *rb;  // forward back left right

    // Powcrtl powcrtl;

    float offset_x;  // 旋转中心距离底盘的距离，云台位于正中心时默认设为0
    float offset_y;

    // sub_pub
    Subscriber *chassis_cmd_suber;
    Publisher *chassis_imu_pub;
    Cmd_chassis *cmd_data;       // 接收到的指令数据
    Upload_chassis upload_data;  // 回传的数据
} Chassis;

Chassis *Chassis_Create(void);
void Chassis_Update(Chassis *obj);
#endif