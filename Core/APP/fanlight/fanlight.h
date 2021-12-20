#ifndef _H_FANLIGHT_H
#define H_FANLIGHT_H
#include "ws2812b.h"
#include "stdint.h"
#define COLOR_SIZE 210

typedef struct fanlight_t {
    ws2812* light;
    color_rgb colors[COLOR_SIZE];
} fanlight;

fanlight* Fanlight_APP_Init();

void FanLight_Update(fanlight* obj);
#endif