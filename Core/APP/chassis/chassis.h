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
#include "VL53L0x.h"
#include "bsp_def.h"
#include "bsp_log.h"
#include "can_motor.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
#include "super_cap_wuli.h"

#pragma pack(1)
typedef struct Chassis_Fly_t {
    int if_fly;
    int wait_calm;
    float wait_calm_cnt;
} Chassis_Fly;

typedef struct Chassis_t {
    BMI088_imu *imu;
    Super_cap_wuli *super_cap;
    // VL53L0x *vl53l0x;
    can_motor *lf;
    can_motor *rf;
    can_motor *lb;
    can_motor *rb;  // forward back left right

    // Chassis_Fly chassis_fly;

    // 旋转中心距离底盘的距离，云台位于正中心时默认设为0
    float offset_x;
    float offset_y;
    // 功率控制相关设定
    float output_limit;   // 轮组最大输出总和限制
    float acc_limit;      // 加速度限制
    float linear_v_base;  // 线速度设定
    float rotate_speed;   // 小陀螺转速设定
    // 过程量
    float proc_target_vx;
    float proc_target_vy;

    // sub_pub
    Subscriber *chassis_cmd_suber;
    Publisher *chassis_imu_pub;
    Cmd_chassis *cmd_data;       // 接收到的指令数据
    Upload_chassis upload_data;  // 回传的数据
} Chassis;
#pragma pack()

Chassis *Chassis_Create(void);
void Chassis_Update(Chassis *obj);
#endif