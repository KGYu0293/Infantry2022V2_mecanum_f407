#ifndef _ROBOT_PARAM_
#define _ROBOT_PARAM_

// 定义主控类型 方便统一板间can通信写法
#define CHASSIS_BOARD
// define GIMBAL_BOARD

#include "stdint.h"

// 各模块pub_sub的参数结构体
// 各部分对外接口统一存放
// 各部分通过pub_sub方式“沟通”的“通讯协议”

// chassis
// vx vy rotate传入时以offset系（一般为云台系）为基准
// 若无云台或云台跟随底盘，offset_angle设置为0即可
// 小陀螺模式下，rotate参数失效，旋转速度由模块内部设定
typedef struct Chassis_param_speed_target_t {
    float vx;            // 单位 mm/s
    float vy;            // 单位 mm/s
    float rotate;        // 单位 度每秒
    float offset_angle;  // vy与底盘正前方的夹角，范围0-360度，方向顺时针，即底盘y方向旋转该角度到达vy方向
} Chassis_param_speed_target;
typedef struct Chassis_param_power_control_t {
    uint8_t power_limit;
    float power_now;
    uint16_t power_buffer;  // 缓冲功率
} Chassis_param_power_control;

typedef struct Chassis_param_t {
    enum { chassis_stop, chassis_run, chassis_rotate_run } mode;
    Chassis_param_speed_target target;
    Chassis_param_power_control power;
} Chassis_param;

// gimbal
typedef struct Gimbal_param_t {
    enum { gimbal_stop, gimbal_run } mode;
    float yaw;
    float pitch;
} Gimbal_param;

// shoot
typedef struct Shoot_param_t {
    enum { shoot_stop, shoot_run } mode;
    enum { not_fire, single, Double, trible, continuous } shoot_command;
    enum { magazine_on, magazine_off } magazine_lid;  // 弹仓盖
    uint16_t bullet_speed;          // 弹速
    float fire_rate;                // 射频（发/秒）
    uint16_t heat_limit_remain;     // 剩余热量，cooling_limit-cooling_heat
} Shoot_param;

#endif