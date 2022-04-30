#include "referee_ui.h"
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
    gd.graphic_tpye = float_t;
    gd.start_x = start_x;
    gd.start_y = start_y;
    gd.start_angle = fontSize;
    gd.radius = ((int32_t)(number * 1000)) >> 22;
    gd.end_x = ((int32_t)(number * 1000)) >> 11;
    gd.end_y = ((int32_t)(number * 1000));
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
    gd.graphic_tpye = int_t;
    gd.start_angle = fontSize;
    gd.start_x = start_x;
    gd.start_y = start_y;
    gd.radius = number >> 22;
    gd.end_x = number >> 11;
    gd.end_y = number;
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
    gd.graphic_tpye = char_t;
    gd.start_angle = fontSize;
    gd.end_angle = ChLength;
    gd.start_x = start_x;
    gd.start_y = start_y;
    return gd;
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

cvector *referee_ui_instances;
void Referee_UI_driver_Init() {
    referee_ui_instances = cvector_create(sizeof(referee_ui *));
}

referee_ui *referee_ui_create(referee_ui_config *config) {
    referee_ui *obj = (referee_ui *)malloc(sizeof(referee_ui));
    memset(obj, 0, sizeof(referee_ui));
    obj->config = *config;
    obj->elements = cvector_create(sizeof(graphic_data));
    obj->send_frame.data_header.sender_ID = config->robot_id;
    obj->send_frame.data_header.receiver_ID = Robot_Client_ID(config->robot_id);
    obj->send_frame.header.SOF = 0xA5;
    obj->send_frame.header.seq = 0;
    cvector_pushback(referee_ui_instances, &obj);
    return obj;
}

//空图形数据
graphic_data empty_data;

void Referee_UI_Loop() {
    for (size_t i = 0; i < referee_ui_instances->cv_len; ++i) {
        referee_ui *obj = *((referee_ui **)cvector_val_at(referee_ui_instances, i));

    }
}

// //添加图片
// void AddGraphic(UI *ui, graphic_config gc) {
//     gd.operate_tpye = 1;
//     circular_queue_push(ui->DataQueue, &gc);
// }
// //修改图片
// void ChangeGraphic(UI *ui, graphic_config gc) {
//     gd.operate_tpye = 2;
//     circular_queue_push(ui->DataQueue, &gc);
// }
// //删除图层
// void DeleteLayer(UI *ui, uint8_t layer) {
//     graphic_config gc;
//     memset(&gc, 0, sizeof(gc));
//     layer = layer;
//     status = 2;
//     circular_queue_push(ui->DataQueue, &gc);
// }
// //删除单个图片
// void DeleteGraphic(UI *ui, uint8_t id, uint8_t isCharacter) {
//     graphic_config gc;
//     memset(&gc, 0, sizeof(gc));
//     if (isCharacter) {
//         char temp[30] = "";
//         gc = Char(id, 0, 0, 0, 0, 0, 0, 0, temp);
//     } else {
//         gc = Circle(id, 0, 0, 0, 0, 0, 0);
//     }
//     gd.operate_tpye = 3;
//     circular_queue_push(ui->DataQueue, &gc);
// }
/*************************************************************************************************/

//内容ID匹配
// void contentIDmatching(UI *ui) {
//     if (ui->tp.Datatype == 0) {
//         switch (ui->tp.num) {
//             case 1:
//                 ui->tp.contentID = 0x0101;
//                 break;
//             case 2:
//                 ui->tp.contentID = 0x0102;
//                 break;
//             case 5:
//                 ui->tp.contentID = 0x0103;
//                 break;
//             case 7:
//                 ui->tp.contentID = 0x0104;
//                 break;
//             default:
//                 return;
//         }
//     } else if (ui->tp.Datatype == 1)
//         ui->tp.contentID = 0x0110;
//     else if (ui->tp.Datatype == 2)
//         ui->tp.contentID = 0x0100;
// }

// void GraphicDataPack(UI *ui) {
//     memset(ui->tx_buff, 0, MAX_SIZE);
//     //数据帧起始字节(固定)
//     ui->tp.frameHeader.SOF = 0xA5;
//     ui->tp.receiverID = IDmatching(ui->tp.senderID);
//     //包序列处理
//     if (ui->tp.frameHeader.seq == 0xff)
//         ui->tp.frameHeader.seq = 0;
//     else
//         ui->tp.frameHeader.seq++;
//     contentIDmatching(ui);
//     ui->tp.cmd_id = 0x0301;
//     ui->tx_buff[0] = ui->tp.frameHeader.SOF;
//     ui->tx_buff[3] = ui->tp.frameHeader.seq;
//     //非字符数据打包
//     if (ui->tp.Datatype == 0) {
//         //确定数据长度
//         ui->tp.frameHeader.data_length = 2 + 2 + 2 + ui->tp.num * 15;
//         memcpy(&ui->tx_buff[1], (uint8_t *)&ui->tp.frameHeader.data_length, sizeof(ui->tp.frameHeader.data_length));
//         ui->tp.frameHeader.CRC8 = CRC8_Modbus_calc(ui->tx_buff, FRAMEHEADER_LEN, crc8_default);
//         ui->tx_buff[4] = ui->tp.frameHeader.CRC8;
//         memcpy(&ui->tx_buff[5], (uint8_t *)&ui->tp.cmd_id, 2);
//         memcpy(&ui->tx_buff[7], (uint8_t *)&ui->tp.contentID, 2);
//         memcpy(&ui->tx_buff[9], (uint8_t *)&ui->tp.senderID, 2);
//         memcpy(&ui->tx_buff[11], (uint8_t *)&ui->tp.receiverID, 2);
//         for (int i = 0; i < ui->tp.num; i++) {
//             memcpy(&ui->tx_buff[13 + 15 * i], &ui->tp.gd[i], sizeof(graphic_data));
//         }
//     }
//     //字符图片类型打包
//     else if (ui->tp.Datatype == 1) {
//         ui->tp.frameHeader.data_length = 51;  // 6+15+30
//         memcpy(&ui->tx_buff[1], (uint8_t *)&ui->tp.frameHeader.data_length, sizeof(ui->tp.frameHeader.data_length));
//         ui->tp.frameHeader.CRC8 = CRC8_Modbus_calc(ui->tx_buff, FRAMEHEADER_LEN, crc8_default);
//         ui->tx_buff[4] = ui->tp.frameHeader.CRC8;
//         memcpy(&ui->tx_buff[5], (uint8_t *)&ui->tp.cmd_id, 2);
//         memcpy(&ui->tx_buff[7], (uint8_t *)&ui->tp.contentID, 2);
//         memcpy(&ui->tx_buff[9], (uint8_t *)&ui->tp.senderID, 2);
//         memcpy(&ui->tx_buff[11], (uint8_t *)&ui->tp.receiverID, 2);
//         memcpy(&ui->tx_buff[13], &ui->tp.gd[0], sizeof(graphic_data));
//         memcpy(&ui->tx_buff[11 + 15], &ui->tp.charData, sizeof(ui->tp.charData));
//     }
//     //删除图层命令打包
//     else if (ui->tp.Datatype == 2) {
//         ui->tp.frameHeader.data_length = 8;  // 6+1+1
//         memcpy(&ui->tx_buff[1], (uint8_t *)&ui->tp.frameHeader.data_length, sizeof(ui->tp.frameHeader.data_length));
//         ui->tp.frameHeader.CRC8 = CRC8_Modbus_calc(ui->tx_buff, FRAMEHEADER_LEN, crc8_default);
//         ui->tx_buff[4] = ui->tp.frameHeader.CRC8;
//         memcpy(&ui->tx_buff[5], (uint8_t *)&ui->tp.cmd_id, 2);
//         memcpy(&ui->tx_buff[7], (uint8_t *)&ui->tp.contentID, 2);
//         memcpy(&ui->tx_buff[9], (uint8_t *)&ui->tp.senderID, 2);
//         memcpy(&ui->tx_buff[11], (uint8_t *)&ui->tp.receiverID, 2);
//         //删除图层,1表示删除图,2表示删除所有
//         ui->tx_buff[13] = 1;
//         //图层信息
//         ui->tx_buff[14] = ui->tp.gd[0].layer;
//     }
//     ui->tp.CRC16 = CRC16_Modbus_calc(ui->tx_buff, FRAMEHEADER_LEN + ui->tp.frameHeader.data_length + CMD_LEN + CRC16_LEN, crc16_default);
//     memcpy(&ui->tx_buff[FRAMEHEADER_LEN + ui->tp.frameHeader.data_length + CMD_LEN - 1], (uint8_t *)&ui->tp.CRC16, CRC16_LEN);
// }

// int SelectDataLength(int num) {
//     switch (num) {
//         case 0:
//             return -1;
//         case 1:
//             return 1;
//         case 2:
//             return 2;
//         case 3:
//             return 2;
//         case 4:
//             return 2;
//         case 5:
//             return 5;
//         case 6:
//             return 5;
//         case 7:
//             return 7;
//     }
// }

//放在10Hz循环中
// void UISendData(UI *ui) {
//     if (ui->DataQueue->cq_len == 0) return;
//     int number = 0;
//     //判断队列前几个图片是什么类型
//     for (int i = 0; i < ui->DataQueue->cq_len; i++) {
//         graphic_config *gc = (graphic_config *)(circular_queue_front(ui->DataQueue) + sizeof(graphic_config) * i);
//         if (gc->status == 1 || gc->status == 2 || i == 7) {
//             break;
//         }
//         number++;
//     }
//     //根据图片类型的不同做出判断
//     if (SelectDataLength(number) != -1) {
//         ui->tp.num = SelectDataLength(number);
//         ui->tp.Datatype = 0;
//         for (int i = 0; i < ui->tp.num; i++) {
//             graphic_config *gc = (graphic_config *)circular_queue_pop(ui->DataQueue);
//             ui->tp.gd[i] = gc->gd;
//         }
//         GraphicDataPack(ui);
//     } else {
//         graphic_config *gc = (graphic_config *)circular_queue_pop(ui->DataQueue);
//         if (gc->status == 1) {
//             ui->tp.num = 1;
//             ui->tp.Datatype = 1;
//             ui->tp.gd[0] = gc->gd;
//             GraphicDataPack(ui);
//         } else if (gc->status == 2) {
//             ui->tp.num = 1;
//             ui->tp.Datatype = 2;
//             ui->tp.gd[0] = gc->gd;
//             GraphicDataPack(ui);
//         }
//     }
//     BSP_UART_Send_DMA(ui->config.bsp_uart_index, ui->tx_buff, FRAMEHEADER_LEN + ui->tp.frameHeader.data_length + CMD_LEN + CRC16_LEN);
// }
