#include "DT7_DR16.h"

#include "bsp.h"
#include "cvector.h"

//键盘对应的通信协议数据
#define Key_W 0x0001
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

cvector *dt7_instances;

void dt7_data_solve(dt7Remote *obj);
void dt7_Rx_Callback(uint8_t uart_index, uint8_t *data, uint32_t len);

void dt7_driver_init(void) {
    dt7_instances = cvector_create(sizeof(dt7Remote *));
    BSP_UART_RegisterRxCallback(UART_REMOTE_PORT, dt7_Rx_Callback);
}

// 构造函数
dt7Remote *dt7_Create(dt7_config *config) {
    dt7Remote *obj = (dt7Remote *)malloc(sizeof(dt7Remote));
    memset(obj, 0, sizeof(dt7Remote));
    obj->config = *config;
    obj->monitor = Monitor_Register(obj->config.lost_callback, 10, obj);
    cvector_pushback(dt7_instances, &obj);
    return obj;
}

void dt7_Rx_Callback(uint8_t uart_index, uint8_t *data, uint32_t len) {
    if (len == DT7_RX_SIZE) {
        for (size_t i = 0; i < dt7_instances->cv_len; i++) {
            dt7Remote *now = *(dt7Remote **)cvector_val_at(dt7_instances, i);
            if (uart_index == now->config.bsp_uart_index) {
                memcpy(now->primary_data, data, DT7_RX_SIZE);
                now->monitor->reset(now->monitor);
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
    // 定义统一输入模式
    if (obj->data.rc.s2 == 2) obj->data.input_mode = RC_Stop;
    if (obj->data.rc.s2 == 3) obj->data.input_mode = RC_Remote;
    if (obj->data.rc.s2 == 1) obj->data.input_mode = RC_MouseKey;

    // 鼠标值解算
    obj->data.mouse.x = ((int16_t)obj->primary_data[6]) | ((int16_t)obj->primary_data[7] << 8);
    obj->data.mouse.y = ((int16_t)obj->primary_data[8]) | ((int16_t)obj->primary_data[9] << 8);
    obj->data.mouse.z = ((int16_t)obj->primary_data[10]) | ((int16_t)obj->primary_data[11] << 8);
    obj->data.mouse.press_l = obj->primary_data[12];
    if(obj->last_data.mouse.press_l && obj->data.mouse.press_l) obj->data.mouse.press_l_cnt = obj->last_data.mouse.press_l_cnt + 1;
    else if(obj->last_data.mouse.press_l && !obj->data.mouse.press_l) obj->data.mouse.press_l_cnt = 0;
    obj->data.mouse.press_r = obj->primary_data[13];
    // 键盘按键解算-是否按下
    uint16_t keyboard = (int16_t)obj->primary_data[14] | ((int16_t)obj->primary_data[15] << 8);
    obj->data.key_down.w = (keyboard & Key_W) > 0;
    obj->data.key_down.s = (keyboard & Key_S) > 0;
    obj->data.key_down.d = (keyboard & Key_D) > 0;
    obj->data.key_down.a = (keyboard & Key_A) > 0;
    obj->data.key_down.shift = (keyboard & Key_Shift) > 0;
    obj->data.key_down.ctrl = (keyboard & Key_ECtrl) > 0;
    obj->data.key_down.q = (keyboard & Key_Q) > 0;
    obj->data.key_down.e = (keyboard & Key_E) > 0;
    obj->data.key_down.r = (keyboard & Key_R) > 0;
    obj->data.key_down.f = (keyboard & Key_F) > 0;
    obj->data.key_down.g = (keyboard & Key_G) > 0;
    obj->data.key_down.z = (keyboard & Key_Z) > 0;
    obj->data.key_down.x = (keyboard & Key_X) > 0;
    obj->data.key_down.c = (keyboard & Key_C) > 0;
    obj->data.key_down.v = (keyboard & Key_V) > 0;
    obj->data.key_down.b = (keyboard & Key_B) > 0;

    // 排除ctrl shift一起按
    if (!(obj->data.key_down.shift && obj->data.key_down.ctrl)) {
        // 键盘按下次数计数 单按 ctrl+ shift+分开处理 不含shift与ctrl本身
        if (obj->data.key_down.shift) {
            if ((obj->data.key_down.w && !obj->last_data.key_down.w)) obj->data.key_shift_press_cnt.w++;
            if ((obj->data.key_down.s && !obj->last_data.key_down.s)) obj->data.key_shift_press_cnt.s++;
            if ((obj->data.key_down.d && !obj->last_data.key_down.d)) obj->data.key_shift_press_cnt.d++;
            if ((obj->data.key_down.a && !obj->last_data.key_down.a)) obj->data.key_shift_press_cnt.a++;
            if ((obj->data.key_down.q && !obj->last_data.key_down.q)) obj->data.key_shift_press_cnt.q++;
            if ((obj->data.key_down.e && !obj->last_data.key_down.e)) obj->data.key_shift_press_cnt.e++;
            if ((obj->data.key_down.r && !obj->last_data.key_down.r)) obj->data.key_shift_press_cnt.r++;
            if ((obj->data.key_down.f && !obj->last_data.key_down.f)) obj->data.key_shift_press_cnt.f++;
            if ((obj->data.key_down.g && !obj->last_data.key_down.g)) obj->data.key_shift_press_cnt.g++;
            if ((obj->data.key_down.z && !obj->last_data.key_down.z)) obj->data.key_shift_press_cnt.z++;
            if ((obj->data.key_down.x && !obj->last_data.key_down.x)) obj->data.key_shift_press_cnt.x++;
            if ((obj->data.key_down.c && !obj->last_data.key_down.c)) obj->data.key_shift_press_cnt.c++;
            if ((obj->data.key_down.v && !obj->last_data.key_down.v)) obj->data.key_shift_press_cnt.v++;
            if ((obj->data.key_down.b && !obj->last_data.key_down.b)) obj->data.key_shift_press_cnt.b++;
        } else if (obj->data.key_down.ctrl) {
            if ((obj->data.key_down.w && !obj->last_data.key_down.w)) obj->data.key_ctrl_press_cnt.w++;
            if ((obj->data.key_down.s && !obj->last_data.key_down.s)) obj->data.key_ctrl_press_cnt.s++;
            if ((obj->data.key_down.d && !obj->last_data.key_down.d)) obj->data.key_ctrl_press_cnt.d++;
            if ((obj->data.key_down.a && !obj->last_data.key_down.a)) obj->data.key_ctrl_press_cnt.a++;
            if ((obj->data.key_down.q && !obj->last_data.key_down.q)) obj->data.key_ctrl_press_cnt.q++;
            if ((obj->data.key_down.e && !obj->last_data.key_down.e)) obj->data.key_ctrl_press_cnt.e++;
            if ((obj->data.key_down.r && !obj->last_data.key_down.r)) obj->data.key_ctrl_press_cnt.r++;
            if ((obj->data.key_down.f && !obj->last_data.key_down.f)) obj->data.key_ctrl_press_cnt.f++;
            if ((obj->data.key_down.g && !obj->last_data.key_down.g)) obj->data.key_ctrl_press_cnt.g++;
            if ((obj->data.key_down.z && !obj->last_data.key_down.z)) obj->data.key_ctrl_press_cnt.z++;
            if ((obj->data.key_down.x && !obj->last_data.key_down.x)) obj->data.key_ctrl_press_cnt.x++;
            if ((obj->data.key_down.c && !obj->last_data.key_down.c)) obj->data.key_ctrl_press_cnt.c++;
            if ((obj->data.key_down.v && !obj->last_data.key_down.v)) obj->data.key_ctrl_press_cnt.v++;
            if ((obj->data.key_down.b && !obj->last_data.key_down.b)) obj->data.key_ctrl_press_cnt.b++;
        } else {
            if ((obj->data.key_down.w && !obj->last_data.key_down.w)) obj->data.key_single_press_cnt.w++;
            if ((obj->data.key_down.s && !obj->last_data.key_down.s)) obj->data.key_single_press_cnt.s++;
            if ((obj->data.key_down.d && !obj->last_data.key_down.d)) obj->data.key_single_press_cnt.d++;
            if ((obj->data.key_down.a && !obj->last_data.key_down.a)) obj->data.key_single_press_cnt.a++;
            if ((obj->data.key_down.q && !obj->last_data.key_down.q)) obj->data.key_single_press_cnt.q++;
            if ((obj->data.key_down.e && !obj->last_data.key_down.e)) obj->data.key_single_press_cnt.e++;
            if ((obj->data.key_down.r && !obj->last_data.key_down.r)) obj->data.key_single_press_cnt.r++;
            if ((obj->data.key_down.f && !obj->last_data.key_down.f)) obj->data.key_single_press_cnt.f++;
            if ((obj->data.key_down.g && !obj->last_data.key_down.g)) obj->data.key_single_press_cnt.g++;
            if ((obj->data.key_down.z && !obj->last_data.key_down.z)) obj->data.key_single_press_cnt.z++;
            if ((obj->data.key_down.x && !obj->last_data.key_down.x)) obj->data.key_single_press_cnt.x++;
            if ((obj->data.key_down.c && !obj->last_data.key_down.c)) obj->data.key_single_press_cnt.c++;
            if ((obj->data.key_down.v && !obj->last_data.key_down.v)) obj->data.key_single_press_cnt.v++;
            if ((obj->data.key_down.b && !obj->last_data.key_down.b)) obj->data.key_single_press_cnt.b++;
        }
    }
}
