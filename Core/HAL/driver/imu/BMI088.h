#ifndef __BMI088_H
#define __BMI088_H
// #include "stm32f4xx_hal.h"
#include "imu_data.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "common.h"
#include "monitor.h"
#include "pid.h"
#include "stdint.h"
typedef struct BMI088_config_t {
    uint8_t bsp_spi_index;
    uint8_t bsp_pwm_heat_index;
    uint8_t bsp_gpio_accel_index;
    uint8_t bsp_gpio_gyro_index;
    float temp_target;
    lost_callback lost_callback;
    int imu_axis_convert[3]; //定义坐标转换矩阵[a1 a2 a3]，公式为 Acc_real[i] = Acc_raw[a_i - 1] * sgn(a_i)，Gyro_real[i] = Gyro_raw[a_i - 1] * sgn(g_i)
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