#ifndef _PWM_SERVO_H_
#define _PWM_SERVO_H_
#include "bsp_pwm.h"
#include "monitor.h"
#include "stdlib.h"
#include "string.h"

typedef enum Servo_Model_e { MODEL_90, MODEL_180, MODEL_270, MODEL_360 } Servo_Model;  //舵机类型

typedef struct Servo_config_t {
    uint8_t bsp_pwm_index;  // 在bsp_pwm中的的端口号
    Servo_Model model;
    uint16_t initial_angle;  // 默认初始位置
} Servo_config;

typedef enum Servo_direction_e { servo_forward, servo_reverse } Servo_direc;
typedef struct Servo_360_control_t {
    Servo_direc direc;
    uint8_t speed;
} Servo_360_control;

typedef struct Servo_t {
    Servo_config config;
    uint32_t ccr;
    // 控制量输入
    uint16_t set_angle;                   // 设定角度（90 180 270舵机）
    Servo_360_control servo_360_control;  // 360舵机
    // int delay_flag;      // 是否需要延时控制的标志 0 不进行延时 1 进行延时
    // uint16_t delay_time;
} Servo;

void Servo_Driver_Init(void);
Servo *Servo_Create(Servo_config *config);

void Servo_Update_ALL(void);

#endif