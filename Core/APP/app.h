#ifndef _APP_H
#define _APP_H
#include "BMI088.h"
#include "buzzer.h"
#include "can_send.h"
#include "can_recv.h"
#include "can_pc.h"
#include "can_motor.h"

//此处预定义所有的外设
extern BMI088_imu* imu;
extern buzzer* internal_buzzer;
extern canpc* pc;

// #TODO to add other motors

void APP_Layer_Init();
void APP_Layer_default_loop();
void APP_Log_Loop();
#endif