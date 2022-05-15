#include <common.h>

float invSqrt(float x) {
    union {
        float y1;
        long y2;
    } data;

    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = x * 0.5F;
    data.y1 = x;
    i = data.y2;
    i = 0x5f3759df - (i >> 1);
    data.y2 = i;
    y = data.y1;
    y = y * (threehalfs - (x2 * y * y));
    y = y * (threehalfs - (x2 * y * y));

    return y;
}