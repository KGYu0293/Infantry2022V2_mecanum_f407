#ifndef _APP_H
#define _APP_H
// 定义主控类型 方便统一板间can通信写法
#define CHASSIS_BOARD
//define GIMBAL_BOARD
// hal layer
#include "BMI088.h"
#include "buzzer.h"
#include "can_send.h"
#include "can_recv.h"
#include "can_pc.h"
#include "can_motor.h"
#include "cvector.h"
#include "referee.h"
// bsp layer
#include "bsp_log.h"

// 重要模块
extern void* robot;
extern void* chassis;
// 共享外设
extern BMI088_imu* imu;
extern buzzer* internal_buzzer;
extern referee* robot_referee;

void APP_Layer_Init();
void APP_Layer_default_loop();
void APP_Log_Loop();
void APP_RobotCmd_Loop();

#endif