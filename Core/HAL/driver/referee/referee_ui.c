#include "referee_ui.h"

#include "soft_crc.h"
#define FRAMEHEADER_LEN 5
#define CMD_LEN 2
#define CRC16_LEN 2
#define MAXGRAPHIC 40

//创建直线
graphic_data Line(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.color = color;
    gd.width = width;
    gd.graphic_tpye = line;
    gd.start_x = start_x;
    gd.start_y = start_y;
    gd.end_x = end_x;
    gd.end_y = end_y;
    return gd;
}

//创建矩形
graphic_data Rectangle(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.width = width;
    gd.color = color;
    gd.graphic_tpye = rectangle;
    gd.start_x = start_x;
    gd.start_y = start_y;
    gd.end_x = end_x;
    gd.end_y = end_y;
    return gd;
}

//创建圆形
graphic_data Circle(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t radius) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.color = color;
    gd.width = width;
    gd.graphic_tpye = circle;
    gd.start_x = x;
    gd.start_y = y;
    gd.radius = radius;
    return gd;
}

//创建椭圆
graphic_data Oval(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t x_center, uint32_t y_center, uint32_t x_halfAxis, uint32_t y_halfAxis) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.color = color;
    gd.width = width;
    gd.graphic_tpye = oval;
    gd.start_x = x_center;
    gd.start_y = y_center;
    gd.end_x = x_halfAxis;
    gd.end_y = y_halfAxis;
    return gd;
}

//创建圆弧
graphic_data Arc(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t start_angle, uint32_t end_angle, uint32_t x_center, uint32_t y_center, uint32_t x_halfAxis, uint32_t y_halfAxis) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.color = color;
    gd.width = width;
    gd.graphic_tpye = arc;
    gd.start_angle = start_angle;
    gd.end_angle = end_angle;
    gd.start_x = x_center;
    gd.start_y = y_center;
    gd.end_x = x_halfAxis;
    gd.end_y = y_halfAxis;
    return gd;
}

//创建浮点数
graphic_data Float(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t fontSize, uint32_t SignificantDigits, uint32_t start_x, uint32_t start_y, float number) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.color = color;
    gd.width = width;
    gd.graphic_tpye = Float_t;
    gd.start_x = start_x;
    gd.start_y = start_y;
    gd.start_angle = fontSize;
    int32_t float_num = number * 1000;
    memcpy(((uint8_t*)&gd) + 11, &float_num, 4);
    return gd;
}

//创建整型数
graphic_data Int(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t fontSize, uint32_t start_x, uint32_t start_y, int number) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.color = color;
    gd.width = width;
    gd.graphic_tpye = Int_t;
    gd.start_angle = fontSize;
    gd.start_x = start_x;
    gd.start_y = start_y;
    memcpy(((uint8_t*)&gd) + 11, &number, 4);
    return gd;
}

//创建字体流
graphic_data Char(uint8_t id, uint32_t layer, uint32_t color, uint32_t width, uint32_t fontSize, uint32_t ChLength, uint32_t start_x, uint32_t start_y) {
    graphic_data gd;
    gd.graphic_name[0] = 0;
    gd.graphic_name[1] = 0;
    gd.graphic_name[2] = id;
    gd.layer = layer;
    gd.color = color;
    gd.width = width;
    gd.graphic_tpye = Char_t;
    gd.start_angle = fontSize;
    gd.end_angle = ChLength;
    gd.start_x = start_x;
    gd.start_y = start_y;
    return gd;
}

void graphic_float_change(graphic_data* graphic, float number) {
    int32_t float_num = number * 1000;
    memcpy(((uint8_t*)graphic) + 11, &float_num, 4);
}
void graphic_int_change(graphic_data* graphic, int number) {
    memcpy(((uint8_t*)graphic) + 11, &number, 4);
}

// 添加图形命令
void add_graphic(referee_ui* obj, graphic_data* graphic) {
    graphic_cmd tmp;
    tmp.data = *graphic;
    tmp.delete_type = 0;
    tmp.data.operate_tpye = 1;  //添加
    referee_ui_add_cmd(obj, &tmp);
}

// 删除图形命令
void del_graphic(referee_ui* obj, graphic_data* graphic) {
    graphic_cmd tmp;
    tmp.data = *graphic;
    tmp.delete_type = 1;
    tmp.data.operate_tpye = 3;  //删除
    referee_ui_add_cmd(obj, &tmp);
}
// 添加字符串命令
void add_text(referee_ui* obj, graphic_data* graphic, char* s, uint8_t len) {
    graphic_cmd tmp;
    tmp.data = *graphic;
    tmp.delete_type = 0;
    tmp.data.operate_tpye = 1;  //添加
    tmp.data.end_angle = len;
    memset(tmp.textdata, 0, 30);
    memcpy(tmp.textdata, s, len);
    referee_ui_add_cmd(obj, &tmp);
}

// 修改图形命令
void modifiy_graphic(referee_ui* obj, graphic_data* graphic) {
    graphic_cmd tmp;
    tmp.data = *graphic;
    tmp.delete_type = 0;
    tmp.data.operate_tpye = 2;  //修改
    referee_ui_add_cmd(obj, &tmp);
}
// 修改字符串命令
void modifiy_text(referee_ui* obj, graphic_data* graphic, char* s, uint8_t len) {
    graphic_cmd tmp;
    tmp.data = *graphic;
    tmp.delete_type = 0;
    tmp.data.operate_tpye = 2;  //修改
    tmp.data.end_angle = len;
    memset(tmp.textdata, 0, 30);
    memcpy(tmp.textdata, s, len);
    referee_ui_add_cmd(obj, &tmp);
}

uint16_t Robot_Client_ID(uint16_t robotID) {
    uint16_t receiverID_client = 0;
    switch (robotID) {
        case RED_HERO:
            receiverID_client = CRED_HERO;
            break;
        case RED_ENGINEER:
            receiverID_client = CRED_ENGINEER;
            break;
        case RED_STANDARD_1:
            receiverID_client = CRED_STANDARD_1;
            break;
        case RED_STANDARD_2:
            receiverID_client = CRED_STANDARD_2;
            break;
        case RED_STANDARD_3:
            receiverID_client = CRED_STANDARD_3;
            break;
        case RED_AERIAL:
            receiverID_client = CRED_AERIAL;
            break;
        case BLUE_HERO:
            receiverID_client = CBLUE_HERO;
            break;
        case BLUE_ENGINEER:
            receiverID_client = CBLUE_ENGINEER;
            break;
        case BLUE_STANDARD_1:
            receiverID_client = CBLUE_STANDARD_1;
            break;
        case BLUE_STANDARD_2:
            receiverID_client = CBLUE_STANDARD_2;
            break;
        case BLUE_STANDARD_3:
            receiverID_client = CBLUE_STANDARD_3;
            break;
        case BLUE_AERIAL:
            receiverID_client = CBLUE_AERIAL;
            break;
    }
    return receiverID_client;
}

cvector* referee_ui_instances;
void Referee_UI_driver_Init() {
    referee_ui_instances = cvector_create(sizeof(referee_ui*));
}

referee_ui* referee_ui_create(referee_ui_config* config) {
    referee_ui* obj = (referee_ui*)malloc(sizeof(referee_ui));
    memset(obj, 0, sizeof(referee_ui));
    obj->config = *config;
    obj->elements = create_circular_queue(sizeof(graphic_cmd), 60);  //最多60个待发送的数据
    obj->send_frame.data_header.sender_ID = config->referee->rx_data.game_robot_state.robot_id;
    obj->send_frame.data_header.receiver_ID = Robot_Client_ID(config->referee->rx_data.game_robot_state.robot_id);
    obj->send_frame.header.SOF = 0xA5;
    obj->send_frame.header.seq = 0;
    obj->send_frame.cmd_id = 0x301;
    cvector_pushback(referee_ui_instances, &obj);
    return obj;
}

void referee_ui_add_cmd(referee_ui* obj, graphic_cmd* element) {
    if (obj->elements->cq_len != obj->elements->cq_max_len) {
        circular_queue_push(obj->elements, element);
    }
}

// 30Hz上限
// 测试发现50Hz能较好的解决丢包问题
void Referee_UI_Loop() {
    for (size_t i = 0; i < referee_ui_instances->cv_len; ++i) {
        referee_ui* obj = *((referee_ui**)cvector_val_at(referee_ui_instances, i));
        obj->send_frame.data_header.sender_ID = obj->config.referee->rx_data.game_robot_state.robot_id;
        obj->send_frame.data_header.receiver_ID = Robot_Client_ID(obj->config.referee->rx_data.game_robot_state.robot_id);
        uint16_t graphic_num = 0;
        while (obj->elements->cq_len > 0) {
            graphic_cmd* now_cmd = circular_queue_front(obj->elements);
            if (now_cmd->delete_type != 0) {  //如果是删除命令，特殊处理
                //如果之前有已经存入send_frame的图片，那就先发送完图片
                if (graphic_num > 0) break;
                obj->send_frame.header.data_length = 6 + 2;
                obj->send_frame.header.CRC8 = CRC8_Modbus_calc(&obj->send_frame.header.SOF, 4, crc8_default);
                obj->send_frame.data_header.data_cmd_id = 0x0100;
                obj->send_frame.data[0] = now_cmd->delete_type;
                obj->send_frame.data[1] = now_cmd->delete_layer;
                uint16_t crc16_now = CRC16_Modbus_calc(&obj->send_frame.header.SOF, 7 + obj->send_frame.header.data_length, crc16_default);
                memcpy(obj->send_frame.data + obj->send_frame.header.data_length - 6, &crc16_now, 2);
                referee_send_ext(obj->config.referee, &obj->send_frame);
                circular_queue_pop(obj->elements);
                break;
            } else if (now_cmd->data.graphic_tpye == Char_t) {  //字符特殊处理
                //如果之前有已经存入send_frame的图片，那就先发送完图片
                if (graphic_num > 0) break;
                obj->send_frame.header.data_length = 6 + 45;
                obj->send_frame.header.CRC8 = CRC8_Modbus_calc(&obj->send_frame.header.SOF, 4, crc8_default);
                obj->send_frame.data_header.data_cmd_id = 0x0110;
                memcpy(obj->send_frame.data, &now_cmd->data, sizeof(graphic_data));
                memcpy(obj->send_frame.data + sizeof(graphic_data), now_cmd->textdata, 30);
                uint16_t crc16_now = CRC16_Modbus_calc(&obj->send_frame.header.SOF, 7 + obj->send_frame.header.data_length, crc16_default);
                memcpy(obj->send_frame.data + obj->send_frame.header.data_length - 6, &crc16_now, 2);
                referee_send_ext(obj->config.referee, &obj->send_frame);
                circular_queue_pop(obj->elements);
                break;
            } else {
                //凑够7个或者直到为空
                memcpy(obj->send_frame.data + graphic_num * sizeof(graphic_data), &now_cmd->data, sizeof(graphic_data));
                circular_queue_pop(obj->elements);
                ++graphic_num;
                if (graphic_num == 7) break;
            }
        }
        if (graphic_num > 0) {
            //没有图片的部分全部设置为0
            memset(obj->send_frame.data + graphic_num * sizeof(graphic_data), 0, sizeof(graphic_data) * (7 - graphic_num));
            if (graphic_num > 5) {
                obj->send_frame.header.data_length = 6 + 105;
                obj->send_frame.data_header.data_cmd_id = 0x0104;
            } else if (graphic_num > 2) {
                obj->send_frame.header.data_length = 6 + 75;
                obj->send_frame.data_header.data_cmd_id = 0x0103;
            } else if (graphic_num > 1) {
                obj->send_frame.header.data_length = 6 + 30;
                obj->send_frame.data_header.data_cmd_id = 0x0102;
            } else {
                obj->send_frame.header.data_length = 6 + 15;
                obj->send_frame.data_header.data_cmd_id = 0x0101;
            }
            obj->send_frame.header.CRC8 = CRC8_Modbus_calc(&obj->send_frame.header.SOF, 4, crc8_default);
            uint16_t crc16_now = CRC16_Modbus_calc(&obj->send_frame.header.SOF, 7 + obj->send_frame.header.data_length, crc16_default);
            memcpy(obj->send_frame.data + obj->send_frame.header.data_length - 6, &crc16_now, 2);
            referee_send_ext(obj->config.referee, &obj->send_frame);
        }
    }
}