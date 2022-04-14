#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_

// 定义主控类型 方便统一板间can通信写法
// 按照要烧录的主控类型 **必须**定义且仅定义一个 另一个注释
#define GIMBAL_BOARD
// #define CHASSIS_BOARD

#include "stdint.h"

// 各模块pub_sub的参数结构体
// 各部分对外接口统一存放
// 各部分通过pub_sub方式“沟通”的“通讯协议”

// 声明所有需要pub/sub的话题（定义只能在.c，在这里extern；否则报错重复定义）
extern const char* chassis_cmd_topic;
extern const char* gimbal_cmd_topic;
extern const char* shoot_cmd_topic;
extern const char* chassis_upload_topic;
extern const char* gimbal_upload_topic;
extern const char* shoot_upload_topic;

#pragma pack(1)

// 对模块是否掉线的定义
typedef enum Module_status_e { module_lost = 0, module_working } Module_status;
// 机器人总模式
typedef enum Robot_mode_e { robot_stop = 0, robot_run } Robot_mode;

// chassis
// vx vy rotate传入时以offset系（一般为云台系）为基准
// 若无云台或云台跟随底盘，offset_angle设置为0即可
// 小陀螺模式下，rotate参数失效，旋转速度由模块内部设定
typedef struct Chassis_param_speed_target_t {
    float vx;            // 单位 mm/s
    float vy;            // 单位 mm/s
    float rotate;        // 单位 度每秒
    float offset_angle;  // vy与底盘正前方的夹角，范围-180 +180度，方向顺时针，即底盘y方向旋转该角度到达vy方向
} Chassis_param_speed_target;
typedef struct Chassis_param_power_control_t {
    uint8_t if_supercap_on;
    uint8_t power_limit;
    float power_now;
    uint16_t power_buffer;  // 缓冲功率
} Chassis_param_power_control;
typedef enum Chassis_mode_e { chassis_stop, chassis_run, chassis_rotate_run, chassis_run_follow_offset } Chassis_mode;

typedef struct Chassis_param_t {
    Chassis_mode mode;
    Chassis_param_speed_target target;
    Chassis_param_power_control power;
} Chassis_param;

// gimbal
typedef struct Gimbal_param_t {
    enum { gimbal_stop, gimbal_run, gimbal_middle } mode;
    float yaw;
    float pitch;

    float rotate_feedforward;
} Gimbal_param;

typedef struct Gimbal_upload_data_t {
    Module_status gimbal_module_status;
    short yaw_encorder;
    float gimbal_imu_euler[3];
} Gimbal_uplode_data;

// shoot
typedef struct Shoot_param_t {
    enum { shoot_stop, shoot_run } mode;
    enum { not_fire, reverse, single, Double, trible, continuous } shoot_command;
    enum { magazine_close,magazine_open } magazine_lid;  // 弹仓盖
    uint16_t bullet_speed;                                // 弹速
    float fire_rate;                                      // 射频（发/秒）
    uint16_t heat_limit_remain;                           // 剩余热量，cooling_limit-cooling_heat
} Shoot_param;

// 板间通信部分
// gimbal output chassis input 云台->底盘数据包
typedef struct board_com_goci_data_t {
    uint8_t if_supercap_on;     // 电容是否开启
    Robot_mode now_robot_mode;  // 遥控器在云台主控 包含stop模式与云台重要模块掉线
    Chassis_mode chassis_mode;
    Chassis_param_speed_target chassis_target;
} board_com_goci_data;
// gimbal input chassis output数据包
typedef struct board_com_gico_data_t {
    Module_status chassis_board_status;  // 同步底盘是否有重要模块掉线
    float gyro_yaw;                      // 将底盘主控的imu数据发到云台
    struct {
        uint16_t bullet_speed_max;   // 弹速
        uint16_t heat_limit_remain;  // 剩余热量
    } shoot_referee_data;
} board_com_gico_data;

#pragma pack()
#endif