#ifndef _BSP_TIM_H
#define _BSP_TIM_H
#include "stdint.h"

void BSP_PWM_Init();
void BSP_PWM_Start(uint8_t pwm_index);
void BSP_PWM_StartCCR_DMA(uint8_t pwm_index, uint32_t* ccr_data, uint16_t len);
void BSP_PWM_Stop(uint8_t pwm_index);
void BSP_PWM_SetCCR(uint8_t pwm_index, uint32_t ccr_value);
void BSP_PWM_SetARR(uint8_t pwm_index, uint32_t arr_value);
uint32_t BSP_PWM_ReadCCR(uint8_t pwm_index);
#endif