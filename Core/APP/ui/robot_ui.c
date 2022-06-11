#include <bsp_random.h>
#include <bsp_time.h>
#include <referee_ui.h>
#include <robot_def.h>
#include <robot_ui.h>
#include <string.h>
#define REPEAT(x) for(int i = 0;i < 1;++i) {x}
// 2s刷新一次
#define UI_REFRESH_INTERVAL 3000

void strset(char* buffer, char* s) {
    memset(buffer, 0, UI_TEXT_BUFFER_SIZE);
    strcpy(buffer, s);
}
//添加图形元素
void Robot_UI_AddElements(robot_ui* obj) {
    // ADD
    add_graphic(obj->ui_sender, &obj->cap_line);
    add_graphic(obj->ui_sender, &obj->cap_rec_outline);
    add_graphic(obj->ui_sender, &obj->cap_int);
    add_graphic(obj->ui_sender, &obj->bat_float);
    add_graphic(obj->ui_sender, &obj->vision_frame);
    
    //指示灯
    add_graphic(obj->ui_sender, &obj->fri_circle);
    add_graphic(obj->ui_sender, &obj->mag_circle);
    add_graphic(obj->ui_sender, &obj->gimbal_circle);
    add_graphic(obj->ui_sender, &obj->chassis_circle);
    add_graphic(obj->ui_sender, &obj->autoaim_circle);
    add_graphic(obj->ui_sender, &obj->power_circle);
    
    
    //文字
    add_text(obj->ui_sender, &obj->fri_text, obj->fri_str, UI_TEXT_BUFFER_SIZE);
    add_text(obj->ui_sender, &obj->mag_text, obj->mag_str, UI_TEXT_BUFFER_SIZE);
    add_text(obj->ui_sender, &obj->gimbal_text, obj->gimbal_str, UI_TEXT_BUFFER_SIZE);
    add_text(obj->ui_sender, &obj->chassis_text, obj->chassis_str, UI_TEXT_BUFFER_SIZE);
    add_text(obj->ui_sender, &obj->autoaim_text, obj->autoaim_str, UI_TEXT_BUFFER_SIZE);
    add_text(obj->ui_sender, &obj->power_text, obj->power_str, UI_TEXT_BUFFER_SIZE);

    //刻度
    add_graphic(obj->ui_sender,&obj->vertical_line);
    add_graphic(obj->ui_sender,&obj->line_15ms_2m);
    add_graphic(obj->ui_sender,&obj->line_15ms_3m);
    add_graphic(obj->ui_sender,&obj->line_15ms_4m);
}

//改变图形元素
void Robot_UI_ModifyElements(robot_ui* obj) {
    //超级电容电量变化
    graphic_int_change(&obj->cap_int, (int)obj->data.cap_percent);
    int cap_len_now = (int)obj->cap_line_len * obj->data.cap_percent / 100.0;
    obj->cap_line.end_x = obj->cap_line.start_x + cap_len_now;
    //电池电压
    graphic_float_change(&obj->bat_float, obj->data.bat_voltage);
    modifiy_graphic(obj->ui_sender, &obj->cap_int);
    modifiy_graphic(obj->ui_sender, &obj->bat_float);
    modifiy_graphic(obj->ui_sender, &obj->cap_line);
    //射速变化
    if(obj->data.bullet_speed == 15)
    {
        obj->vertical_line.start_x = 945;
        obj->vertical_line.end_x = 945;
        //15ms,2m
        obj->line_15ms_2m.start_x = 840;
        obj->line_15ms_2m.start_y = 416;
        obj->line_15ms_2m.end_x = 1050;
        obj->line_15ms_2m.end_y = 416;

        //15ms,3m
        obj->line_15ms_3m.start_x = 860;
        obj->line_15ms_3m.start_y = 405;
        obj->line_15ms_3m.end_x = 1030;
        obj->line_15ms_3m.end_y = 405;
        //18ms,4m
        obj->line_15ms_4m.start_x = 880;
        obj->line_15ms_4m.start_y = 400;
        obj->line_15ms_4m.end_x = 1010;
        obj->line_15ms_4m.end_y = 400;

        modifiy_graphic(obj->ui_sender,&obj->vertical_line);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_2m);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_3m);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_4m);
    }
    else if(obj->data.bullet_speed == 18)
    {
        obj->vertical_line.start_x = 945;
        obj->vertical_line.end_x = 945;
        //18ms,3m
        obj->line_15ms_2m.start_x = 840;
        obj->line_15ms_2m.start_y = 435;
        obj->line_15ms_2m.end_x = 1050;
        obj->line_15ms_2m.end_y = 435;
        //18ms,4m
        obj->line_15ms_3m.start_x = 860;
        obj->line_15ms_3m.start_y = 430;
        obj->line_15ms_3m.end_x = 1030;
        obj->line_15ms_3m.end_y = 430;
        //18ms,5m
        obj->line_15ms_4m.start_x = 880;
        obj->line_15ms_4m.start_y = 420;
        obj->line_15ms_4m.end_x = 1010;
        obj->line_15ms_4m.end_y = 420;

        modifiy_graphic(obj->ui_sender,&obj->vertical_line);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_2m);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_3m);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_4m);
    }
    else if(obj->data.bullet_speed == 30)
    {
        //竖直线右移
        obj->vertical_line.start_x = 960;
        obj->vertical_line.end_x = 960;
        //30ms,3m
        obj->line_15ms_2m.start_x = 840;
        obj->line_15ms_2m.start_y = 485;
        obj->line_15ms_2m.end_x = 1050;
        obj->line_15ms_2m.end_y = 485;
        //30ms,5m
        obj->line_15ms_3m.start_x = 860;
        obj->line_15ms_3m.start_y = 480;
        obj->line_15ms_3m.end_x = 1030;
        obj->line_15ms_3m.end_y = 480;
        //30ms,7m
        obj->line_15ms_4m.start_x = 880;
        obj->line_15ms_4m.start_y = 475;
        obj->line_15ms_4m.end_x = 1010;
        obj->line_15ms_4m.end_y = 475;

        modifiy_graphic(obj->ui_sender,&obj->vertical_line);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_2m);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_3m);
        modifiy_graphic(obj->ui_sender,&obj->line_15ms_4m);
    }

    //摩擦轮变化
    if (obj->data.fri_mode == shoot_run) {
        obj->fri_circle.color = Green;
        strset(obj->fri_str, "FRI:ON");
    } else {
        obj->fri_circle.color = Orange;
        strset(obj->fri_str, "FRI:OFF");
    }

    //弹仓盖变化
    if (obj->data.mag_mode == magazine_open) {
        obj->mag_circle.color = Orange;
        strset(obj->mag_str, "HATCH:OPEN");
    } else {
        obj->mag_circle.color = Green;
        strset(obj->mag_str, "HATCH:CLOSE");
    }

    //云台模式
    if (obj->data.gimbal_mode == gimbal_middle) {
        obj->gimbal_circle.color = Pink;
        strset(obj->gimbal_str, "GIMBAL:MIDDLE");
    } else if (obj->data.gimbal_mode == gimbal_stop) {
        obj->gimbal_circle.color = White;
        strset(obj->gimbal_str, "GIMBAL:OFF");
    } else {
        obj->gimbal_circle.color = Green;
        strset(obj->gimbal_str, "GIMBAL:NORM");
    }

    //底盘模式
    if (obj->data.chassis_mode == chassis_stop) {
        obj->chassis_circle.color = White;
        strset(obj->chassis_str, "CHASSIS:OFF");
    } else if (obj->data.chassis_mode == chassis_run_follow_offset) {
        obj->chassis_circle.color = Green;
        strset(obj->chassis_str, "CHASSIS:FOLLOW");
    } else if (obj->data.chassis_mode == chassis_run) {
        obj->chassis_circle.color = Pink;
        strset(obj->chassis_str, "CHASSIS:IND");
    } else {
        obj->chassis_circle.color = Purplish_Red;
        strset(obj->chassis_str, "CHASSIS:ROT");
    }

    // 功率模式
    if (obj->data.power_mode == chassis_dispatch_mild){
        obj->power_circle.color = Green;
        strset(obj->power_str, "POWER:MILD");
    } else if (obj->data.power_mode == chassis_dispatch_shift){
        obj->power_circle.color = Orange;
        strset(obj->power_str, "POWER:SHIFT");
    } else if (obj->data.power_mode == chassis_dispatch_climb){
        obj->power_circle.color = Orange;
        strset(obj->power_str, "POWER:CLIMB");
    } else if (obj->data.power_mode == chassis_dispatch_fly){
        obj->power_circle.color = Purplish_Red;
        strset(obj->power_str, "POWER:FLY");
    } else if (obj->data.power_mode == chassis_dispatch_without_acc_limit){
        obj->power_circle.color = Cyan;
        strset(obj->power_str, "POWER:ACC");
    }


    //自瞄模式
    if (!obj->data.pc_online) {
        obj->autoaim_circle.color = Orange;
        obj->vision_frame.width = 0;
        strset(obj->autoaim_str, "AUTOAIM:OFFLINE");
    } else if (obj->data.autoaim_mode == auto_aim_off) {
        obj->autoaim_circle.color = White;
        obj->vision_frame.width = 0;
        strset(obj->autoaim_str, "AUTOAIM:OFF");
    } else if (obj->data.autoaim_mode == auto_aim_normal) {
        obj->autoaim_circle.color = Green;
        obj->vision_frame.width = 1;
        strset(obj->autoaim_str, "AUTOAIM:NORM");
    } else if (obj->data.autoaim_mode == auto_aim_buff_small) {
        obj->autoaim_circle.color = Pink;
        obj->vision_frame.width = 1;
        strset(obj->autoaim_str, "AUTOAIM:SMALL");
    } else if (obj->data.autoaim_mode == auto_aim_buff_big) {
        obj->autoaim_circle.color = Cyan;
        obj->vision_frame.width = 1;
        strset(obj->autoaim_str, "AUTOAIM:BIG");
    }
    obj->vision_frame.color = obj->data.vision_has_taget ? Green: White;

    modifiy_graphic(obj->ui_sender, &obj->vision_frame);
    if(obj->data.fri_mode != obj->last_data.fri_mode) modifiy_graphic(obj->ui_sender, &obj->fri_circle);
    if(obj->data.mag_mode != obj->last_data.mag_mode) modifiy_graphic(obj->ui_sender, &obj->mag_circle);
    if(obj->data.gimbal_mode != obj->last_data.gimbal_mode) modifiy_graphic(obj->ui_sender, &obj->gimbal_circle);
    if(obj->data.chassis_mode != obj->last_data.chassis_mode) modifiy_graphic(obj->ui_sender, &obj->chassis_circle);
    if(obj->data.autoaim_mode != obj->last_data.autoaim_mode || obj->data.pc_online != obj->last_data.pc_online) modifiy_graphic(obj->ui_sender, &obj->autoaim_circle);
    if(obj->data.power_mode != obj->last_data.power_mode) modifiy_graphic(obj->ui_sender, &obj->power_circle);
    if(obj->data.fri_mode != obj->last_data.fri_mode) REPEAT(modifiy_text(obj->ui_sender, &obj->fri_text, obj->fri_str, 20);)
    if(obj->data.mag_mode != obj->last_data.mag_mode) REPEAT(modifiy_text(obj->ui_sender, &obj->mag_text, obj->mag_str, 20);)
    if(obj->data.gimbal_mode != obj->last_data.gimbal_mode) REPEAT(modifiy_text(obj->ui_sender, &obj->gimbal_text, obj->gimbal_str, 20);)
    if(obj->data.chassis_mode != obj->last_data.chassis_mode) REPEAT(modifiy_text(obj->ui_sender, &obj->chassis_text, obj->chassis_str, 20);)
    if(obj->data.autoaim_mode != obj->last_data.autoaim_mode || obj->data.pc_online != obj->last_data.pc_online) REPEAT(modifiy_text(obj->ui_sender, &obj->autoaim_text, obj->autoaim_str, 20);)
    if(obj->data.power_mode != obj->last_data.power_mode) REPEAT(modifiy_text(obj->ui_sender, &obj->power_text, obj->power_str, 20);)
    if(obj->data.bullet_speed != obj->last_data.power_mode) REPEAT(modifiy_graphic(obj->ui_sender,&obj->line_15ms_2m);)
    if(obj->data.bullet_speed != obj->last_data.power_mode) REPEAT(modifiy_graphic(obj->ui_sender,&obj->line_15ms_3m);)
    if(obj->data.bullet_speed != obj->last_data.power_mode) REPEAT(modifiy_graphic(obj->ui_sender,&obj->line_15ms_4m);)
    if(obj->data.bullet_speed != obj->last_data.power_mode) REPEAT(modifiy_graphic(obj->ui_sender,&obj->vertical_line);)
    obj->last_data = obj->data;
}

robot_ui* Create_Robot_UI(robot_ui_config* _config) {
    robot_ui* obj = (robot_ui*)malloc(sizeof(robot_ui));
    memset(obj, 0, sizeof(robot_ui));
    obj->config = *_config;
    referee_ui_config sender_config;
    sender_config.referee = obj->config.referee;
    // sender_config.robot_id = obj->config.robot_id;
    //创建referee_ui 对象
    obj->ui_sender = referee_ui_create(&sender_config);

    //创建各种 UI 实体对象
    obj->cap_line_len = 1210 - 710;
    obj->cap_rec_outline = Rectangle(0, 0, Green, 2, 710, 100, 1210, 150);
    obj->cap_line = Line(1, 0, Yellow, 50, 710, 125, 960, 125);
    // obj->cap_float = Float(2, 0, Yellow, 2, 30, 1, 1225, 140, 50.0);
    obj->cap_int = Int(2, 0, Yellow, 2, 30, 1225, 140, 50);
    obj->bat_float = Float(3, 0, RedBlue, 2, 30, 1, 1225, 100, 24.0);
    obj->vision_frame = Rectangle(4, 0, White, 0, 820, 430, 1100, 650);

    //摩擦轮
    obj->fri_circle = Circle(5, 0, Green, 8, 150, 731, 10);
    obj->fri_text = Char(6, 0, White, 3, 20, 20, 180, 740);
    strset(obj->fri_str, "FRI:OFF");

    //弹仓
    obj->mag_circle = Circle(7, 0, Green, 8, 150, 681, 10);
    obj->mag_text = Char(8, 0, White, 3, 20, 20, 180, 690);
    strset(obj->mag_str, "HATCH:CLOSE");

    //云台
    obj->gimbal_circle = Circle(9, 0, Green, 8, 150, 631, 10);
    obj->gimbal_text = Char(10, 0, White, 3, 20, 20, 180, 640);
    strset(obj->gimbal_str, "GIMBAL:NORM");

    //底盘
    obj->chassis_circle = Circle(11, 0, Green, 8, 150, 581, 10);
    obj->chassis_text = Char(12, 0, White, 3, 20, 20, 180, 590);
    strset(obj->chassis_str, "CHASSIS:FOLLOW");

    //自瞄
    obj->autoaim_circle = Circle(13, 0, Green, 8, 150, 531, 10);
    obj->autoaim_text = Char(14, 0, White, 3, 20, 20, 180, 540);
    strset(obj->autoaim_str, "AUTOAIM:OFF");
    //功率
    obj->power_circle = Circle(15, 0, Green, 8, 1550, 731, 10);
    obj->power_text = Char(16, 0, White, 3, 20, 20, 1580, 740);
    strset(obj->power_str, "POWER:MILD");

    
    //刻度

    obj->vertical_line = Line(17,1,Purplish_Red,1,945,300,945,550);
    obj->line_15ms_2m = Line(18,1,Yellow,1,840,416,1050,416);
    obj->line_15ms_3m = Line(19,1,Yellow,1,860,405,1030,405);
    obj->line_15ms_4m = Line(20,1,Yellow,1,880,400,1010,400);


    //初始化percent
    obj->data.cap_percent = 0.0;
    obj->data.bat_voltage = 0.0;

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