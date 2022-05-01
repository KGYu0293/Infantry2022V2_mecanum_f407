#ifndef _REFEREE_UI_H
#define _REFEREE_UI_H
#include "circular_queue.h"
#include "referee.h"
#include "referee_def.h"
#pragma pack(1)

//颜色
typedef enum Color_t {
    RedBlue = 0,   //红蓝主色
    Yellow,        //黄色
    Green,         //绿色
    Orange,        //橙色
    Purplish_Red,  //紫红色
    Pink,          //粉色
    Cyan,          //青色
    Black,         //黑色
    White,         //白色
} Color;

//图形类型
typedef enum Graphic_type_t {
    line = 0,   //直线
    rectangle,  //矩形
    circle,     //圆S
    oval,       //椭圆
    arc,        //圆弧
    float_t,    //浮点数
    int_t,      //整型数
    char_t,     //字符
} Graphic_type;

//图形数据  字体大小与线宽比例推荐为10：1
typedef struct graphic_data_t {
    uint8_t graphic_name[3];    //图形名字 在删除，修改等操作中，作为客户端的索引
    uint32_t operate_tpye : 3;  //图形操作  0：空操作 1：增加  2：修改  3：删除
    uint32_t graphic_tpye : 3;  //图形类型
    uint32_t layer : 4;         //图层 0-9
    uint32_t color : 4;         //颜色
    uint32_t start_angle : 9;   //起始角度 [0,360]
    uint32_t end_angle : 9;     //终止角度 [0,360]
    uint32_t width : 10;        //线宽
    uint32_t start_x : 11;      //起点x坐标
    uint32_t start_y : 11;      //起点y坐标
    uint32_t radius : 10;       //字体大小或者半径
    uint32_t end_x : 11;        //终点x坐标
    uint32_t end_y : 11;        //终点y坐标
} graphic_data;

#pragma pack()

/*
HAL层referee_ui对象自动管理graphic_cmd并发送
referee_ui只有 管理数据发送/图形创建工具函数 等基础功能
*/
typedef struct graphic_cmd_t {
    graphic_data data;
    uint8_t textdata[30];
    uint8_t delete_type;
    uint8_t delete_layer;
} graphic_cmd;

typedef struct referee_ui_config_t {
    Referee* referee;  // referee_ui 依赖referee对象
    uint16_t robot_id;
} referee_ui_config;

typedef struct referee_ui_t {
    circular_queue* elements;  //待发送的图形元素
    referee_ui_config config;
    ext_robot_interact_frame send_frame;
} referee_ui;

// 20Hz调用该函数
void Referee_UI_Loop();

void Referee_UI_driver_Init();
referee_ui* referee_ui_create(referee_ui_config* config);
// 放入图形命令
void referee_ui_add_cmd(referee_ui* obj, graphic_cmd* element);

// 绘图工具函数
graphic_data Line(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
graphic_data Rectangle(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
graphic_data Circle(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t radius);
graphic_data Oval(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t x_center, uint32_t y_center, uint32_t x_halfAxis, uint32_t y_halfAxis);
graphic_data Arc(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t start_angle, uint32_t end_angle, uint32_t x_center, uint32_t y_center, uint32_t x_halfAxis, uint32_t y_halfAxis);
graphic_data Float(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t fontSize, uint32_t SignificantDigits, uint32_t start_x, uint32_t start_y, float number);
graphic_data Int(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t fontSize, uint32_t start_x, uint32_t start_y, int number);
graphic_data Char(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t fontSize, uint32_t ChLength, uint32_t start_x, uint32_t start_y);

void graphic_float_change(graphic_data* graphic, float number);
void graphic_int_change(graphic_data* graphic, int number);
#endif