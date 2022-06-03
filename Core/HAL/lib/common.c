#include <common.h>
#include <math.h>
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

int sgn(int x){
    return x == 0 ? 0 : x > 0 ? 1 : -1;
}

int fsgn(float x) {
    return (x != 0.0f ? (x < 0.0f ? -1 : 1) : 0);
}

float sgn_like(float x, float d) {
    if (fabs(x) >= d)
        return fsgn(x);
    else
        return x / d;
}