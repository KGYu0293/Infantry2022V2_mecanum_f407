#ifndef _HAL_LAYER_H
#define _HAL_LAYER_H
#include "can_motor.h"


void HAL_Layer_Init();
void HAL_Layer_Default_Loop();
void HAL_Imu_Loop();
void HAL_Motor_Calc_Loop();
void HAL_Super_cap_wuli_Loop();
#endif