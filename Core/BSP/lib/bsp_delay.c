#include "bsp_delay.h"

#include "bsp_def.h"
#include "tim.h"

void bsp_delay_us(uint16_t nus) {
    __HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
    __HAL_TIM_ENABLE(DLY_TIM_Handle);
    while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus) {
    }
    __HAL_TIM_DISABLE(DLY_TIM_Handle);
}

void bsp_delay_ms(uint16_t nms) {
    for (int i = 0; i < nms; ++i) {
        bsp_delay_us(1000);
    }
}