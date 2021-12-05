#include "hal.h"
#include "BMI088.h"
#include "crc16.h"
#include "can_recv.h"
#include "can_pc.h"
#include "bsp_delay.h"

void HAL_Layer_Init(){
    CRC16_Init();
    BMI088_Driver_Init();
    CanRecv_Driver_Init();
}

void HAL_Default_Loop(){

}

void HAL_Imu_Loop(){
    BMI088_Update_All();
}

void HAL_Motor_Calc_Loop(){
    
}