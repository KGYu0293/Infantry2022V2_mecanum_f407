#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include <mrac.h>
#include <pid.h>

enum controller_type_e { PID_MODEL = 0,
                         MRAC_MODEL };

enum controller_depth { CURRENT_CONTROL = 0,
                        SPEED_CONTROL,
                        POS_CONTROL };
typedef struct controller_config_t {
    enum controller_type_e control_type;  //控制器算法类型
    enum controller_depth control_depth;  //控制器深度
    //使用MRAC时填写初始化结构体，MRAC只能使用位置控制模式
    mrac_2d mrac_2d_init;
    //使用PID控制时填写双环PID配置结构体
    struct PID_config_t speed_pid_config;
    struct PID_config_t position_pid_config;
} controller_config;

typedef struct controller_t {
    controller_config config;
    mrac_2d mrac_2d_data;
    pid pid_speed_data;
    pid pid_pos_data;
    float output;
    float ref_speed;
    float ref_position;
    float fdb_speed;
    float fdb_position;
} controller;

void controller_calc(controller* obj);
controller* create_controller(controller_config* _config);
#endif