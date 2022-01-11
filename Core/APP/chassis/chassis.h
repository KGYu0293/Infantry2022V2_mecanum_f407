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
#include "app.h"

// 底盘传入参数结构体
// vx vy rotate传入时以offset系（一般为云台系）为基准
// 若无云台或云台跟随底盘，offset_angle设置为0即可
// 小陀螺模式下，rotate参数失效，旋转速度由模块内部设定
typedef struct Chassis_param_t {
    enum { stop, run, rotate_run } mode;
    float vx;// 单位 mm/s
    float vy;// 单位 mm/s
    float rotate;// 单位 度每秒
    float offset_angle; // vy与底盘正前方的夹角，范围0-360度，方向顺时针，即底盘y方向旋转该角度到达vy方向
} Chassis_param;

// 定义chassis所需的外设，整合成一个结构体
typedef struct chassis_t {
    can_motor *lf;
    can_motor *rf;
    can_motor *lb;
    can_motor *rb;  // forward back left right
    
    float offset_x;  // 旋转中心距离底盘的距离，云台位于正中心时默认设为0
    float offset_y;
} Chassis;

Chassis *Chassis_Create(void);
void Chassis_Update(Chassis *obj,Chassis_param param);
#endif