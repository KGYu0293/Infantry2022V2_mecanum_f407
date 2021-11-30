#ifndef __COMMON_H
#define __COMMON_H
#include "stdint.h"
#define pi 3.1415926f
#define RAD2DEG 180.0f / pi

void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);
float invSqrt(float x);
#endif