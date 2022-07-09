#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include <adrc.h>
#include <mrac.h>
#include <pid.h>
#include <smc.h>

enum controller_type_e { PID_MODEL = 0, MRAC_MODEL, ADRC_MODEL, SMC_MODEL };

enum controller_depth { CURRENT_CONTROL = 0, SPEED_CONTROL, POS_CONTROL };

#pragma pack(1)
typedef struct controller_config_t {
    enum controller_type_e control_type;  //控制器算法类型
    enum controller_depth control_depth;  //控制器深度
    //使用MRAC时填写初始化结构体，MRAC只能使用位置控制模式
    Mrac_config mrac_config;
    //使用PID控制时填写双环PID配置结构体
    struct PID_config_t speed_pid_config;
    struct PID_config_t position_pid_config;
    //使用ADRC控制时填写ADRC配置结构体
    ADRC_Config_t speed_adrc_config;
    ADRC_Config_t pos_adrc_config;
    // 使用滑膜控制时配置结构体，目前只使用速度环控制
    Smc_config speed_smc_config;
} controller_config;

typedef struct controller_t {
    controller_config config;
    mrac_2d mrac_2d_data;
    pid pid_speed_data;
    pid pid_pos_data;
    ADRC_t adrc_speed_data;
    ADRC_t adrc_pos_data;
    Smc smc_speed_data;
    float output;
    float ref_speed;
    float ref_position;
    float fdb_speed;
    float fdb_position;
} controller;
#pragma pack()

void controller_calc(controller* obj);
controller* create_controller(controller_config* _config);
#endif