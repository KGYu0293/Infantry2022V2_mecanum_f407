/**
 * @file           : mrac.c
 * @brief          :
 * @Author         : lwy
 * @Date           : 2022-04-10 09:36
 * @LastEditTime   : 2022-04-28 17:34
 * @Note           :
 * @Copyright(c)   : 哈尔滨工业大学（深圳）南工骁鹰机器人队版权所有 Critical HIT copyrighted
 */

#include "mrac.h"

#include "math.h"
#include "common.h"

struct ramp_t {
    int count;
    int scale;
    float out;
    void (*init)(struct ramp_t *ramp, int scale);
    float (*calc)(struct ramp_t *ramp);
};

// int ssgn(float x);
float sat(float x, float lower_bound, float upper_bound);
float sgn_like(float x, float d);
void RampInit(struct ramp_t *ramp, int scale);
float RampCalc(struct ramp_t *ramp);
float FlickFunc(int period, float scale, unsigned char mode);
float TriangularWave(float max, float speed);
float mrac_index(float a, float b);

// int ssgn(float x) {
//     if (x > 0)
//         return 1;
//     else if (x < 0)
//         return -1;
//     else
//         return 0;
// }

float sat(float x, float lower_bound, float upper_bound) {
    if (lower_bound > upper_bound) return 0;
    if (x > upper_bound)
        return upper_bound;
    else if (x < lower_bound)
        return lower_bound;
    else
        return x;
}


float sgn_like(float x, float d) {
    if (fabs(x) >= d)
        return fsgn(x);
    else
        return x / d;
}


void RampInit(struct ramp_t *ramp, int scale) {
    ramp->count = 0;      
    ramp->scale = scale; 
    ramp->out = 0; 
}


float RampCalc(struct ramp_t *ramp) {
    if (ramp->scale <= 0)
        return 0;
    ramp->count++; 
    if (ramp->count >= ramp->scale)
        ramp->count = ramp->scale; 

    ramp->out = ramp->count / ((float)ramp->scale);  
    return ramp->out;
}


float FlickFunc(int period, float scale, unsigned char mode) {
    static int flick_time = 0;
    if (mode == 0) {
        flick_time = 0;
        return 0;
    }
    if (flick_time > period || flick_time < 0)  
        flick_time = 0;
    else
        flick_time++;

    if (flick_time >= 0 && flick_time < period / 2)
        return (scale - 4.0f * flick_time * scale / period);
    else if (flick_time >= period / 2 && flick_time < period)
        return (4.0f * flick_time * scale / period - 3.0f * scale);
    else
        return 0;
}


float TriangularWave(float max, float speed) {
    static int i, x = 0; 
    if (i == max)
        x = 1;
    else if (i == -max)
        x = 0;
    if (i < max && x == 0)
        i += speed;
    else if (x == 1 && i > -max)
        i -= speed;
    return i;
}

float mrac_index(float a, float b) {
    float number = 1.0;

    if (b == 0)
        return 1;
    else if (b < 0) {
        for (int i = 0; i < -b; i++)
            number *= a;
        return 1 / number;
    } else {
        for (int i = 0; i < b; i++)
            number *= a;
        return number;
    }
}