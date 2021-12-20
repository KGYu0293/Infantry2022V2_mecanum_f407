#ifndef _H_WS2812B_H
#define _H_WS2812B_H

#include "stdint.h"


typedef struct color_rgb_t {
    uint8_t r, g, b;
} color_rgb;

typedef struct ws2812_config_t{
    uint8_t pwm_id;
    uint32_t max_len;
} ws2812_config;

typedef struct ws2812_t{
	uint8_t* buffer;
	uint8_t* frame_start;
    uint16_t send_len;
	ws2812_config config;
} ws2812;

extern color_rgb red;
extern color_rgb green;
extern color_rgb purple;
extern color_rgb blue;
extern color_rgb off;

ws2812* ws2812_create(ws2812_config* config);
void ws2812_close_all(ws2812* obj);
void ws2812_set_all(ws2812* obj, color_rgb color);
void ws2812_set_array(ws2812* obj,color_rgb* color_buffer,uint32_t len);

// void rgb_2_raw(color_raw* buff, color_rgb);
// void array_2_raw(color_raw* data_buff, color_rgb* color_buff, uint16_t len);
// void color_array_init(void);
// void calc_flow_array(uint16_t index);
// void ws2812_send_frame(void);
// void ws2812_set_all(color_rgb);
// void ws2812_set_array(color_rgb* color_buff);
// void ws2812_close_all(void);


// extern frame_buf_ST frame;
// extern color_rgb lights[PIXEL_MAX];
// extern uint16_t flow_index;

#endif