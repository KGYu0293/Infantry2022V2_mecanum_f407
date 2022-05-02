#include <bsp_random.h>
#include <bsp_time.h>
#include <referee_ui.h>
#include <robot_ui.h>
// 2s刷新一次
#define UI_REFRESH_INTERVAL 2000
//添加图形元素
void Robot_UI_AddElements(robot_ui* obj) {
    add_graphic(obj->ui_sender, &obj->test);
    add_graphic(obj->ui_sender, &obj->test_line);
    add_graphic(obj->ui_sender, &obj->test_circle);
    add_graphic(obj->ui_sender, &obj->test_float);
    add_text(obj->ui_sender, &obj->test_str, "CHASSIS", 7);
}

//改变图形元素
void Robot_UI_ModifyElements(robot_ui* obj) {
    graphic_int_change(&obj->test, GetRand_Int() % 114514);
    modifiy_graphic(obj->ui_sender, &obj->test);
    graphic_float_change(&obj->test_float, GetRand_Range(-100, 200));
    modifiy_graphic(obj->ui_sender, &obj->test_float);
    obj->test_circle = Circle(3, 0, Green, 2, 240, 800, (uint32_t)GetRand_Range(30, 60));
    modifiy_graphic(obj->ui_sender, &obj->test_circle);
    static uint8_t str_op = 0;
    str_op ^= 1;
    modifiy_text(obj->ui_sender, &obj->test_str, str_op ? "CHASSIS" : "GIMBAL", str_op ? 7 : 6);
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
    obj->test = Int(1, 0, Purplish_Red, 1, 30, 240, 640, 114514);
    obj->test_line = Line(2, 0, Green, 2, 640, 240, 640, 500);
    obj->test_circle = Circle(3, 0, Green, 2, 240, 800, 50);
    obj->test_float = Float(4, 0, Yellow, 1, 30, 2, 240, 700, 114.514);
    obj->test_str = Char(5, 0, Green, 1, 30, 10, 460, 700);

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