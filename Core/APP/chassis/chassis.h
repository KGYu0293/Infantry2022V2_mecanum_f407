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
#include "pub_sub.h"
#include "robot_param.h"
#include "stdint.h"
#include "bsp_log.h"
#include "can_motor.h"

// 定义chassis所需的外设，整合成一个结构体
typedef struct chassis_t {
    can_motor *lf;
    can_motor *rf;
    can_motor *lb;
    can_motor *rb;  // forward back left right
    
    float offset_x;  // 旋转中心距离底盘的距离，云台位于正中心时默认设为0
    float offset_y;

    Subscriber* chassis_cmd_suber;
} Chassis;

Chassis *Chassis_Create(void);
void Chassis_Update(Chassis *obj);
#endif