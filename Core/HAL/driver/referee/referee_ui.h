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
typedef struct {
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

typedef struct {
    graphic_data data;
    char* textdata;
} graphic_element;

//发送数据包
typedef struct {
    uint8_t num;  //表示图片的数量
    frame_header frameHeader;
    uint16_t receiverID;
    uint16_t senderID;
    uint16_t contentID;
    graphic_data gd[7];  //单次发送图片最多7个
    uint8_t Datatype;    // 0表示非字符，1表示字符，2表示删除图层
    uint16_t cmd_id;
    uint16_t CRC16;
    char charData[30];
} Txpack;

typedef struct {
    uint8_t bsp_uart_index;
    uint16_t robot_id;
} UI_config;

typedef struct {
    cvector* elements;
    UI_config config;
} UI;

#pragma pack()