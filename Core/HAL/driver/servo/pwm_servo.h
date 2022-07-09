#ifndef _PWM_SERVO_H_
#define _PWM_SERVO_H_
#include "bsp_pwm.h"
#include "monitor.h"
#include "stdlib.h"
#include "string.h"

typedef enum Servo_Model_e { MODEL_SPEED, MODEL_POS } Servo_Model;  //舵机类型

#pragma pack(1)
typedef struct Servo_config_t {
    uint8_t bsp_pwm_index;  // 在bsp_pwm中的的端口号
    Servo_Model model;
    uint16_t initial_angle;  // 默认初始位置
    uint16_t max_angle;      // 最大可调节角度 如常见的90 180 270
} Servo_config;

typedef enum Speed_servo_direction_e { servo_hold ,servo_forward, servo_reverse } Servo_direc;
typedef struct Speed_servo_control_t {
    Servo_direc direc;
    uint8_t speed;
} Speed_servo_control;

typedef struct Servo_t {
    Servo_config config;
    // 控制量输入
    uint16_t pos_servo_control;               // 设定角度（角度式）
    Speed_servo_control speed_servo_control;  // 360舵机（速度式）
} Servo;
#pragma pack()

void Servo_Driver_Init(void);
Servo *Servo_Create(Servo_config *config);

void Servo_Update_ALL(void);

#endif