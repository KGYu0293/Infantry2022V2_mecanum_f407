#include "bsp_pwm.h"

#include "bsp_def.h"
#include "dma.h"
#include "tim.h"

typedef struct BSP_PWM_Typedef_t {
    TIM_HandleTypeDef* base;
    uint16_t channel;
} BSP_PWM_Typedef;

BSP_PWM_Typedef pwm_ports[DEVICE_PWM_CNT];
extern DMA_HandleTypeDef PWM_DMA_1;
void BSP_PWM_Init() {
    pwm_ports[0].base = PWM_0_BASE;
    pwm_ports[0].channel = PWM_0_CHANNEL;

    pwm_ports[1].base = PWM_1_BASE;
    pwm_ports[1].channel = PWM_1_CHANNEL;

    pwm_ports[2].base = PWM_2_BASE;
    pwm_ports[2].channel = PWM_2_CHANNEL;
    //处理HAL库DMA BUG
    HAL_DMA_DeInit(&PWM_DMA_1);
    HAL_DMA_Init(&PWM_DMA_1);
    HAL_TIM_PWM_Stop_DMA(PWM_2_BASE, PWM_2_CHANNEL);
}

void BSP_PWM_Start(uint8_t pwm_index) { HAL_TIM_PWM_Start(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel); }

void BSP_PWM_Stop(uint8_t pwm_index) { HAL_TIM_PWM_Stop(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel); }

void BSP_PWM_SetCCR(uint8_t pwm_index, uint32_t ccr_value) { __HAL_TIM_SetCompare(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel, ccr_value); }

void BSP_PWM_SetARR(uint8_t pwm_index, uint32_t arr_value) { __HAL_TIM_SetAutoreload(pwm_ports[pwm_index].base, arr_value); }

void BSP_PWM_StartCCR_DMA(uint8_t pwm_index, uint32_t* ccr_data, uint16_t len) { HAL_TIM_PWM_Start_DMA(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel, ccr_data, len); }

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    for (size_t i = 0; i < DEVICE_PWM_CNT; ++i) {
        if (htim == pwm_ports[i].base) {
            HAL_TIM_PWM_Stop_DMA(pwm_ports[i].base, pwm_ports[i].channel);
        }
    }
}