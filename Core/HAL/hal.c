#include "hal.h"

#include "BMI088.h"
#include "DT7_DR16.h"
#include "bsp_delay.h"
#include "buzzer.h"
#include "can_motor.h"
#include "can_pc.h"
#include "can_recv.h"
#include "monitor.h"
#include "pub_sub.h"
#include "referee.h"
#include "referee_ui.h"
#include "pwm_servo.h"
#include "soft_crc.h"
#include "super_cap_wuli.h"

void HAL_Layer_Init() {
    Monitor_Init();
    soft_crc_Init();
    Buzzer_Driver_Init();
    BMI088_Driver_Init();
    CanRecv_Driver_Init();
    Can_Motor_Driver_Init();
    dt7_driver_init();
    referee_driver_init();
    Referee_UI_driver_Init();
    SubPub_Init();
    Super_cap_wuli_Driver_Init();
    Servo_Driver_Init();
}

void HAL_Layer_Default_Loop() {
    Servo_Update_ALL();
}

void HAL_Imu_Loop() { BMI088_Update_All(); }

void HAL_Motor_Calc_Loop() { Can_Motor_Calc_Send(); }

void HAL_Super_cap_wuli_Loop() { Super_cap_wuli_Send(); }

void HAL_Monitor_Loop() { Monitor_Loop(); }

void HAL_Buzzer_Loop() { Buzzer_Loop(); }

void HAL_Referee_Recv_Loop() { referee_recv_loop(); }

void HAL_Referee_Send_Loop() { Referee_UI_Loop(); }