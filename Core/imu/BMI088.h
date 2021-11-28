#ifndef __BMI088_H
#define __BMI088_H
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "tim.h"
#include "common.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

//此处欧拉角定义为 绕固定参考坐标轴旋转Z-Y-X 也就是 yaw pitch roll
typedef struct _imu_data{
    float accel[3];
    float gyro[3];
    float euler[3];
    float euler_deg[3];
} imu_data;
typedef struct _BMI088_imu
{
    uint8_t init_error;
    uint8_t bias_init_success;
    
    
    imu_data data;
    float gyrobias[3];
    float temp;
    float temp_target;
    
    TIM_HandleTypeDef* HEAT_PWM_BASE;
    SPI_HandleTypeDef* SPI_PORT;
    GPIO_TypeDef* ACCEL_NS_BASE;
    GPIO_TypeDef* GYRO_NS_BASE;
    uint16_t HAET_PWM_CHANNEL;
    uint16_t ACCEL_NS_PIN;
    uint16_t GYRO_NS_PIN;

    struct PID_t heat_pid;
    MahonyAHRS mahony_solver;
    MadgwickAHRS madgwick_solver;
} BMI088_imu;

extern BMI088_imu imu;

uint8_t BMI088_init(BMI088_imu* obj);
void BMI088_Update(BMI088_imu* obj);
#endif