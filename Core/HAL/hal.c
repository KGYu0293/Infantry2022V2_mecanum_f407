#include "hal.h"
#include "BMI088.h"
#include "soft_crc.h"
#include "can_recv.h"
#include "can_pc.h"
#include "can_motor.h"
#include "super_cap_wuli.h"
#include "bsp_delay.h"
#include "DT7_DR16.h"
#include "monitor.h"
#include "pub_sub.h"
#include "referee.h"

void HAL_Layer_Init(){
    Monitor_Init();
    soft_crc_Init();
    BMI088_Driver_Init();
    CanRecv_Driver_Init();
    Can_Motor_Driver_Init();
    dt7_driver_init();
    referee_driver_init();
    SubPub_Init();
    Super_cap_wuli_Driver_Init();
}

void HAL_Layer_Default_Loop(){

}

void HAL_Imu_Loop(){
    BMI088_Update_All();
}

void HAL_Motor_Calc_Loop(){
    Can_Motor_Calc_Send();
}

void HAL_Super_cap_wuli_Loop(){
    Super_cap_wuli_Send();
}

void HAL_Monitor_Loop(){
    Monitor_Loop();
}