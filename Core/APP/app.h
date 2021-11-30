#ifndef _APP_H
#define _APP_H
#include "BMI088.h"
#include "buzzer.h"

//此处预定义所有的外设
extern BMI088_imu* imu;
extern buzzer* internal_buzzer;

//定义外设变量的配置文件
extern BMI088_config internal_imu_config;
extern buzzer_config internal_buzzer_config;

void APP_Layer_Init();
void APP_Layer_default_loop();
void APP_Log_Loop();
#endif