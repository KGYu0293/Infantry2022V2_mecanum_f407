#ifndef __BMI088_H
#define __BMI088_H
// #include "stm32f4xx_hal.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "common.h"
#include "monitor.h"
#include "pid.h"
#include "stdint.h"

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_pwm.h"

//此处欧拉角定义为 绕固定参考坐标轴旋转X-Y-Y 也就是 pitch roll yaw
typedef struct imu_data_t {
    float accel[3];// ZYX加速度
    //按陀螺仪原始角速度定义 pitch roll yaw
    float gyro[3];// 角速度 度每秒
    float euler[3];// 欧拉角 弧度 
    float euler_deg[3];// 欧拉角 角度 
    float euler_8192[3];// 欧拉角 编码器版 0-8192
    
    int round;
    float yaw_8192_real;
} imu_data;

typedef struct BMI088_config_t {
    uint8_t bsp_spi_index;
    uint8_t bsp_pwm_heat_index;
    uint8_t bsp_gpio_accel_index;
    uint8_t bsp_gpio_gyro_index;
    float temp_target;
    lost_callback lost_callback;
} BMI088_config;

typedef struct BMI088_imu_t {
    uint8_t init_error;
    uint8_t bias_init_success;

    imu_data data;
    float gyrobias[3];
    float temp;

    BMI088_config config;

    struct PID_t heat_pid;
    MahonyAHRS mahony_solver;
    MadgwickAHRS madgwick_solver;
    monitor_item* monitor;
} BMI088_imu;


void BMI088_Driver_Init();
void BMI088_Update_All();
// Public methods
BMI088_imu* BMI088_Create(BMI088_config* config);
void BMI088_Update(BMI088_imu* obj);


#endif