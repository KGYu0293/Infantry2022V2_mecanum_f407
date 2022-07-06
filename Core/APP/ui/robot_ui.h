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
    uint8_t power_mode;        // 底盘功率模式
    uint8_t vision_has_taget;  // 找到目标
    float cap_percent;
    float bat_voltage;
    float bullet_speed;         //子弹弹速
    float angle;                //云台与底盘夹角(弧度)
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

    //功率模式
    graphic_data power_circle;
    graphic_data power_text;
    char power_str[UI_TEXT_BUFFER_SIZE];

    //自瞄指示框
    graphic_data vision_frame;
    int cap_line_len;

    ui_status data;
    ui_status last_data;

     //瞄准标尺(命名规则：弹速-瞄准距离)不同弹速的瞄准线长不同
    graphic_data vertical_line;
    graphic_data line_15ms_2m;//对应于18ms：3  30ms:3
    graphic_data line_15ms_3m;//对应于18ms：4  30ms:5
    graphic_data line_15ms_4m;//对应于18ms：5  30ms:7
    
    //云台与底盘夹角提示
    graphic_data angle_chassis;//绘制底盘
    graphic_data angle_gimbal;//绘制云台
    
    // UI刷新时间戳
    uint32_t time_refresh;
} robot_ui;

robot_ui* Create_Robot_UI(robot_ui_config* _config);
void Robot_UI_update(robot_ui* obj);
#endif