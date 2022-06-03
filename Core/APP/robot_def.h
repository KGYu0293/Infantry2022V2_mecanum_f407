#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_

// 定义主控类型 方便统一板间can通信写法
// 按照要烧录的主控类型 **必须**定义且仅定义一个 另一个注释
#define GIMBAL_BOARD
// #define CHASSIS_BOARD

#include "stdint.h"
#include "stdlib.h"
// 部分外设数据定义
#include "imu_data.h"
#include "referee_def.h"

// 机器人结构参数定义
#include "robot_struct.h"

// 各模块pub_sub的参数结构体
// 各部分对外接口统一存放
// 各部分通过pub_sub方式“沟通”的“通讯协议”

#pragma pack(1)
/** 机器人模式定义 **/

// 对模块是否掉线的定义
typedef enum Module_status_e { module_lost = 0,
                               module_working } Module_status;

// 机器人总模式
typedef enum Robot_mode_e { robot_stop = 0,
                            robot_run } Robot_mode;

// 底盘运行模式
typedef enum Chassis_mode_e {
    chassis_stop = 0,
    chassis_run,               // 底盘云台分离模式
    chassis_rotate_run,        // 小陀螺模式
    chassis_run_follow_offset  // 底盘跟随云台模式
} Chassis_mode;

// 底盘调度模式
typedef enum Chassis_dispatch_mode_e {
    chassis_dispatch_mild = 0,           // 限制加速度 不消耗电容
    chassis_dispatch_without_acc_limit,  // 无加速度限制 不消耗电容
    chassis_dispatch_shift,              // 无加速度限制 耗电容
    chassis_dispatch_climb,              // 爬坡
    chassis_dispatch_fly                 // 飞坡
} Chassis_dispatch_mode;

// 拨弹轮运行模式
typedef enum Bullet_mode_e {
    bullet_holdon = 0,
    bullet_reverse,    // 反转，卡弹处理
    bullet_single,     // 单发
    bullet_double,     // 双发
    bullet_trible,     // 三发
    bullet_continuous  // 连发
} Bullet_mode;

// 发射机构运行模式
typedef enum Shoot_mode_e {
    shoot_stop = 0,  // 关闭发射机构
    shoot_run
} Shoot_mode;

// 弹仓盖模式
typedef enum Magazine_mode_e {
    magazine_close,  // 开弹仓
    magazine_open    // 关弹仓
} Magazine_mode;

// 云台模式
typedef enum Gimbal_mode_e {
    gimbal_stop = 0,
    gimbal_run,     // 正常云台模式
    gimbal_middle,  // 云台归中，跟随底盘模式
} Gimbal_mode;

// 自瞄模式
typedef enum AutoAim_mode_e {
    auto_aim_off = 0,     // 关闭自瞄
    auto_aim_normal,      // 正常自瞄
    auto_aim_buff_small,  // 小能量机关
    auto_aim_buff_big     // 大能量机关
} AutoAim_mode;

/** 机器人模块控制量定义 **/

// 对底盘速度的控制量
/* vx vy rotate传入时以offset系（一般为云台系）为基准
 * 若无云台或云台跟随底盘，offset_angle设置为0即可
 * 小陀螺模式下，rotate参数失效，旋转速度由模块内部设定
 */
typedef struct Cmd_chassis_speed_t {
    float vx;            // 单位 mm/s
    float vy;            // 单位 mm/s
    float rotate;        // 单位 度每秒
    float offset_angle;  // vy与底盘正前方的夹角，范围-180 +180度，方向顺时针，即底盘y方向旋转该角度到达vy方向
} Cmd_chassis_speed;

// 对底盘功率的控制量
typedef struct Cmd_chassis_power_t {
    // uint8_t if_consume_supercap;  //是否消耗电容
    Chassis_dispatch_mode dispatch_mode;
    uint16_t power_limit;
    float power_now;
    uint16_t power_buffer;  // 缓冲功率
} Cmd_chassis_power;

// 对底盘模块的控制量（总）
typedef struct Cmd_chassis_t {
    Chassis_mode mode;
    Cmd_chassis_speed target;
    Cmd_chassis_power power;
} Cmd_chassis;

// 对发射机构的控制量
typedef struct Cmd_shoot_t {
    Shoot_mode mode;
    Bullet_mode bullet_mode;    // 发射模式
    Magazine_mode mag_mode;     // 弹仓盖
    uint16_t bullet_speed;      // 弹速
    float fire_rate;            // 射频（发/秒）
    int16_t heat_limit_remain;  // 剩余热量，cooling_limit-cooling_heat
} Cmd_shoot;

// 对云台的控制量
typedef struct Cmd_gimbal_t {
    Gimbal_mode mode;
    float yaw;
    float pitch;

    float rotate_feedforward;  // 小陀螺前馈
} Cmd_gimbal;

// 云台回传cmd的数据
typedef struct Upload_gimbal_t {
    Module_status gimbal_status;
    imu_data* gimbal_imu;
    short* yaw_encorder;
} Upload_gimbal;

// 底盘模块回传cmd的数据
typedef struct Upload_chassis_t {
    Module_status chassis_status;
    float chassis_supercap_percent;
    float chassis_battery_voltage;
    imu_data* chassis_imu;
} Upload_chassis;

// 板间通信定义
// 云台->底盘数据包
typedef struct Gimbal_board_send_t {
    uint8_t now_robot_mode;            // 遥控器在云台主控 包含stop模式与云台重要模块掉线
    uint8_t chassis_mode;              // 底盘模式
    uint8_t chassis_dispatch_mode;     // 底盘功率模式
    Cmd_chassis_speed chassis_target;  // 底盘速度控制
    uint8_t autoaim_mode;              // UI所需自瞄数据
    uint8_t pc_online;                 // UI所需PC是否在线
    uint8_t gimbal_mode;               // UI所需云台数据
    uint8_t mag_mode;                  // UI所需弹仓盖数据
    uint8_t fri_mode;                  // UI所需摩擦轮数据
    uint8_t vision_has_target;         // 自瞄是否检测到目标
} Gimbal_board_send_data;

// 云台<-底盘数据包
typedef struct Chassis_board_send_t {
    uint8_t chassis_board_status;  // 同步底盘是否有重要模块掉线
    float gyro_yaw;                // 将底盘主控的imu数据发到云台
    struct {
        uint16_t bullet_speed_max;  // 弹速
        int16_t heat_limit_remain;  // 剩余热量
    } shoot_referee_data;
    uint8_t robot_id;
    uint8_t chassis_supercap_percent;
} Chassis_board_send_data;

#pragma pack()
#endif