#ifndef __COMMON_H
#define __COMMON_H
#include "stdint.h"
#define pi 3.1415926f
#define RAD2DEG 180.0f / pi
#define DEG2RAD pi / 180.0f

float invSqrt(float x);
int sgn(int x);
int fsgn(float x);
float sgn_like(float x, float d);
#endif