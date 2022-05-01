#include <referee_ui.h>
#include <robot_ui.h>

robot_ui* Create_Robot_UI(robot_ui_config* _config) {
    robot_ui* obj = (robot_ui*)malloc(sizeof(robot_ui));
    memset(obj, 0, sizeof(robot_ui));
    obj->config = *_config;
    referee_ui_config sender_config;
    sender_config.referee = obj->config.referee;
    sender_config.robot_id = obj->config.robot_id;
    obj->ui_sender = referee_ui_create(&sender_config);
    obj->test = Int(1, 0, Purplish_Red, 10, 3, 640, 640, 114514);
    obj->test.operate_tpye = 1;
    graphic_cmd add_cmd;
    add_cmd.data = obj->test;
    add_cmd.delete_type = 0;
    referee_ui_add_cmd(obj->ui_sender, &add_cmd);
    return obj;
}