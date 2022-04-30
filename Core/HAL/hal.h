#ifndef _HAL_LAYER_H
#define _HAL_LAYER_H

void HAL_Layer_Init();
void HAL_Layer_Default_Loop();
void HAL_Imu_Loop();
void HAL_Buzzer_Loop();
void HAL_Motor_Calc_Loop();
void HAL_Monitor_Loop();
void HAL_Super_cap_wuli_Loop();
void HAL_Referee_Recv_Loop();
#endif