#include <stdlib.h>
#include <ws2812b.h>

#include "bsp_pwm.h"
#include "string.h"
// frame_buf_ST frame;
// color_rgb lights[PIXEL_MAX];

// 定义数据0和数据1的CCR占空比

#define BIT_1 140u
#define BIT_0 70u
//预定义颜色
color_rgb red = {20, 0, 0};
color_rgb green = {0, 10, 0};
color_rgb purple = {14, 4, 23};
color_rgb blue = {0, 0, 20};
color_rgb off = {0, 0, 0};




//工具函数，转换rgb值到一个长度24的uint16数组（TIMER的CCR寄存器为16位）
void rgb_2_raw(uint16_t* buff, color_rgb color) {
    for (uint16_t i = 0; i < 8; ++i) {
        buff[i] = ((color.g << i) & (0x80)) ? BIT_1 : BIT_0;
        buff[i + 8] = ((color.r << i) & (0x80)) ? BIT_1 : BIT_0;
        buff[i + 16] = ((color.b << i) & (0x80)) ? BIT_1 : BIT_0;
    }
}
//转换一个数列
void array_2_raw(uint16_t* buff, color_rgb* color_buff, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        rgb_2_raw(buff + i * 24, color_buff[i]);
    }
}

ws2812* ws2812_create(ws2812_config* config) {
    ws2812* obj = (ws2812*)malloc(sizeof(ws2812));
    obj->config = *config;
    obj->send_len = 3 + 24 * obj->config.max_len + 1;
    obj->buffer = malloc(obj->send_len * 2);
    memset(obj->buffer, 0, obj->send_len * 2);
    obj->frame_start = obj->buffer + 3;
    return obj;
}

void ws2812_send_frame(ws2812* obj) {
    BSP_PWM_StartCCR_DMA(obj->config.pwm_id, (uint32_t*)obj->buffer, obj->send_len);
}

void ws2812_set_all(ws2812* obj, color_rgb color) {
    for (size_t i = 0; i < obj->config.max_len; ++i) {
        rgb_2_raw(obj->frame_start + i * 24, color);
    }
    ws2812_send_frame(obj);
}

void ws2812_close_all(ws2812* obj) {
    ws2812_set_all(obj, off);
}

void ws2812_set_array(ws2812* obj,color_rgb* color_buffer,uint32_t len){
    array_2_raw(obj->frame_start,color_buffer,len);
    ws2812_send_frame(obj);
}