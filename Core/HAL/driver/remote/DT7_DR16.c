#include "DT7_DR16.h"

#include "bsp_uart.h"
#include "cvector.h"

#define CHx_BIAS 1024  //通道中间值为1024

#define Key_W 0x0001  //键盘对应的通信协议数据
#define Key_S 0x0002
#define Key_D 0x0004
#define Key_A 0x0008
#define Key_Shift 0x0010
#define Key_ECtrl 0x0020
#define Key_Q 0x0040
#define Key_E 0x0080
#define Key_R 0x0100
#define Key_F 0x0200
#define Key_G 0x0400
#define Key_Z 0x0800
#define Key_X 0x1000
#define Key_C 0x2000
#define Key_V 0x4000
#define Key_B 0x8000

void dt7_data_solve(dt7Remote *obj);
cvector *dt7_instances;

void dt7_driver_init(void) {
    dt7_instances = cvector_create(sizeof(dt7Remote *));
    BSP_UART_RegisterRxCallback(UART_REMOTE_PORT, dt7_IDLE_Callback);
}

// 构造函数
dt7Remote *dt7_Create(dt7_config *config) {
    dt7Remote *obj = (dt7Remote *)malloc(sizeof(dt7Remote));
    obj->config = *config;
    cvector_pushback(dt7_instances, &obj);
    return obj;
}

void dt7_IDLE_Callback(uint8_t uart_index, uint8_t *data, uint32_t len) {
    if (len == DT7_RX_SIZE) {
        for (size_t i = 0; i < dt7_instances->cv_len; i++) {
            dt7Remote *now = *(dt7Remote **)cvector_val_at(dt7_instances, i);
            if (uart_index == now->config.bsp_uart_index) {
                memcpy(now->primary_data, data, DT7_RX_SIZE);
                dt7_data_solve(now);
            }
        }
    }
}

// 遥控器数据解算
void dt7_data_solve(dt7Remote *obj) {
    // last赋值
    obj->last_data = obj->data;
    // 拨杆值解算
    obj->data.rc.ch0 = ((int16_t)obj->primary_data[0] | ((int16_t)obj->primary_data[1] << 8)) & 0x07FF;
    obj->data.rc.ch1 = (((int16_t)obj->primary_data[1] >> 3) | ((int16_t)obj->primary_data[2] << 5)) & 0x07FF;
    obj->data.rc.ch2 = (((int16_t)obj->primary_data[2] >> 6) | ((int16_t)obj->primary_data[3] << 2) | ((int16_t)obj->primary_data[4] << 10)) & 0x07FF;
    obj->data.rc.ch3 = (((int16_t)obj->primary_data[4] >> 1) | ((int16_t)obj->primary_data[5] << 7)) & 0x07FF;
    obj->data.rc.ch4 = ((int16_t)obj->primary_data[16] | ((int16_t)obj->primary_data[17] << 8)) & 0x07FF;
    obj->data.rc.s1 = ((obj->primary_data[5] >> 4) & 0x000C) >> 2;
    obj->data.rc.s2 = ((obj->primary_data[5] >> 4) & 0x0003);
    // 鼠标值解算
    obj->data.mouse.x = ((int16_t)obj->primary_data[6]) | ((int16_t)obj->primary_data[7] << 8);
    obj->data.mouse.y = ((int16_t)obj->primary_data[8]) | ((int16_t)obj->primary_data[9] << 8);
    obj->data.mouse.z = ((int16_t)obj->primary_data[10]) | ((int16_t)obj->primary_data[11] << 8);
    obj->data.mouse.press_l = obj->primary_data[12];
    obj->data.mouse.press_r = obj->primary_data[13];
    // 键盘按键解算
    uint16_t keyboard = (int16_t)obj->primary_data[14] | ((int16_t)obj->primary_data[15] << 8);
    obj->data.key_down.w = keyboard & Key_W;
    obj->data.key_down.s = keyboard & Key_S;
    obj->data.key_down.d = keyboard & Key_D;
    obj->data.key_down.a = keyboard & Key_A;
    obj->data.key_down.shift = keyboard & Key_Shift;
    obj->data.key_down.ctrl = keyboard & Key_ECtrl;
    obj->data.key_down.q = keyboard & Key_Q;
    obj->data.key_down.e = keyboard & Key_E;
    obj->data.key_down.r = keyboard & Key_R;
    obj->data.key_down.f = keyboard & Key_F;
    obj->data.key_down.g = keyboard & Key_G;
    obj->data.key_down.z = keyboard & Key_Z;
    obj->data.key_down.x = keyboard & Key_X;
    obj->data.key_down.c = keyboard & Key_C;
    obj->data.key_down.v = keyboard & Key_V;
    obj->data.key_down.b = keyboard & Key_B;
}
