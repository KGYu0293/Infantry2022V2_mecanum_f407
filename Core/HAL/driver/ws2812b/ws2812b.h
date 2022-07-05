#ifndef _H_WS2812B_H
#define _H_WS2812B_H

#include "stdint.h"

#pragma pack(1)
typedef struct color_rgb_t {
    uint8_t r, g, b;
} color_rgb;

typedef struct ws2812_config_t {
    uint8_t pwm_id;
    uint32_t max_len;
} ws2812_config;

typedef struct ws2812_t {
    uint16_t* buffer;
    uint16_t* frame_start;
    uint16_t send_len;
    ws2812_config config;
} ws2812;
#pragma pack()

extern color_rgb red;
extern color_rgb green;
extern color_rgb purple;
extern color_rgb blue;
extern color_rgb off;

ws2812* ws2812_create(ws2812_config* config);
void ws2812_close_all(ws2812* obj);
void ws2812_set_all(ws2812* obj, color_rgb color);
void ws2812_set_array(ws2812* obj, color_rgb* color_buffer, uint32_t len);

#endif