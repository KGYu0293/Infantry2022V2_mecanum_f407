#ifndef __BMI088_H
#define __BMI088_H
// #include "stm32f4xx_hal.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "common.h"
#include "monitor.h"
#include "exceptions.h"
#include "pid.h"
#include "stdint.h"

//此处欧拉角定义为 绕固定参考坐标轴旋转Z-Y-X 也就是 yaw pitch roll
typedef struct imu_data_t {
    float accel[3];
    float gyro[3];
    float euler[3];
    float euler_deg[3];
} imu_data;

typedef struct BMI088_config_t {
    uint8_t bsp_spi_index;
    uint8_t bsp_pwm_heat_index;
    uint8_t bsp_gpio_accel_index;
    uint8_t bsp_gpio_gyro_index;
    float temp_target;
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