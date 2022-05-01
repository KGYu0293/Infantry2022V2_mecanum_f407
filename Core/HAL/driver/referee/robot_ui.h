#ifndef _ROBOT_UI_H
#define _ROBOT_UI_H
#include <referee_ui.h>

typedef struct robot_ui_config_t {
    Referee* referee;
    uint8_t robot_id;
} robot_ui_config;

typedef struct robot_ui_t {
    robot_ui_config config;
    referee_ui* ui_sender;
    graphic_data test;
} robot_ui;

robot_ui* Create_Robot_UI(robot_ui_config* _config);
void Robot_UI_update(robot_ui* obj);
#endif