#include <bsp_random.h>
#include <bsp_time.h>
#include <referee_ui.h>
#include <robot_ui.h>
// 2s刷新一次
#define UI_REFRESH_INTERVAL 2000
//添加图形元素
void Robot_UI_AddElements(robot_ui* obj) {
    add_graphic(obj->ui_sender, &obj->cap_line);
    add_graphic(obj->ui_sender, &obj->cap_rec_outline);
    add_graphic(obj->ui_sender, &obj->cap_float);
}

//改变图形元素
void Robot_UI_ModifyElements(robot_ui* obj) {
    graphic_float_change(&obj->cap_float,obj->cap_percent);
    int cap_len_now = (int)obj->cap_line_len * obj->cap_percent / 100.0;
    obj->cap_line.end_x = obj->cap_line.start_x + cap_len_now;
    modifiy_graphic(obj->ui_sender, &obj->cap_float);
    modifiy_graphic(obj->ui_sender, &obj->cap_line);
}

robot_ui* Create_Robot_UI(robot_ui_config* _config) {
    robot_ui* obj = (robot_ui*)malloc(sizeof(robot_ui));
    memset(obj, 0, sizeof(robot_ui));
    obj->config = *_config;
    referee_ui_config sender_config;
    sender_config.referee = obj->config.referee;
    sender_config.robot_id = obj->config.robot_id;
    //创建referee_ui 对象
    obj->ui_sender = referee_ui_create(&sender_config);

    //创建各种 UI 实体对象
    obj->cap_line_len = 1210 - 710;
    obj->cap_rec_outline = Rectangle(0, 0, Green, 2, 710, 100, 1210, 150);
    obj->cap_line = Line(1, 1, Yellow, 50, 710, 125, 960, 125);
    obj->cap_float = Float(2, 0, Yellow, 2, 30, 1, 1225, 140, 50.0);

    //初始化percent
    obj->cap_percent = 0.0;

    Robot_UI_AddElements(obj);
    obj->time_refresh = BSP_sys_time_ms();
    return obj;
}

void Robot_UI_update(robot_ui* obj) {
    //等待之前的UI命令发送完成
    if (obj->ui_sender->elements->cq_len > 0) return;
    uint32_t time_now = BSP_sys_time_ms();
    //刷新UI，解决断电重启和丢包等问题
    if (time_now - obj->time_refresh > UI_REFRESH_INTERVAL) {
        obj->time_refresh = time_now;
        // Robot_UI_Clear() 清空画布，其实没有必要
        Robot_UI_AddElements(obj);  //添加UI元素
    } else {
        Robot_UI_ModifyElements(obj);
    }
}