#ifndef _ROBOT_UI_H
#define _ROBOT_UI_H
#include <referee_ui.h>
#define UI_TEXT_BUFFER_SIZE 20
typedef struct robot_ui_config_t {
    Referee* referee;
} robot_ui_config;

typedef struct ui_status_t {
    uint8_t autoaim_mode;      // UI所需自瞄数据
    uint8_t pc_online;         // UI所需PC是否在线
    uint8_t gimbal_mode;       // UI所需云台数据
    uint8_t mag_mode;          // UI所需弹仓盖数据
    uint8_t fri_mode;          // UI所需摩擦轮数据
    uint8_t chassis_mode;      // UI所需底盘数据
    uint8_t vision_has_taget;  //找到目标
    float cap_percent;
    float bat_voltage;
} ui_status;

typedef struct robot_ui_t {
    robot_ui_config config;
    referee_ui* ui_sender;
    //电容相关
    graphic_data cap_line;
    graphic_data cap_rec_outline;
    graphic_data cap_int;

    //电池电压
    graphic_data bat_float;

    //摩擦轮
    graphic_data fri_circle;
    graphic_data fri_text;
    char fri_str[UI_TEXT_BUFFER_SIZE];

    //弹仓
    graphic_data mag_circle;
    graphic_data mag_text;
    char mag_str[UI_TEXT_BUFFER_SIZE];

    //云台
    graphic_data gimbal_circle;
    graphic_data gimbal_text;
    char gimbal_str[UI_TEXT_BUFFER_SIZE];

    //底盘
    graphic_data chassis_circle;
    graphic_data chassis_text;
    char chassis_str[UI_TEXT_BUFFER_SIZE];

    //自瞄模式
    graphic_data autoaim_circle;
    graphic_data autoaim_text;
    char autoaim_str[UI_TEXT_BUFFER_SIZE];

    //自瞄指示框
    graphic_data vision_frame;
    int cap_line_len;

    ui_status data;
    // UI刷新时间戳
    uint32_t time_refresh;
} robot_ui;

robot_ui* Create_Robot_UI(robot_ui_config* _config);
void Robot_UI_update(robot_ui* obj);
#endif