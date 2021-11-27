#include <common.h>
#include <tim.h>

#define DLY_TIM_Handle (&htim13)

void delay_us(uint16_t nus)
{
    __HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
    __HAL_TIM_ENABLE(DLY_TIM_Handle);
    while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus)
    {
    }
    __HAL_TIM_DISABLE(DLY_TIM_Handle);
}

void delay_ms(uint16_t nms)
{
    for (int i = 0; i < nms; ++i)
    {
        delay_us(1000);
    }
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}