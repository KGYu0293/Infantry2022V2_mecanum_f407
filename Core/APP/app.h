#ifndef _APP_H
#define _APP_H

// bsp layer
#include "bsp_log.h"
// hal layer
#include "BMI088.h"
#include "buzzer.h"
#include "can_send.h"
#include "can_recv.h"
#include "can_pc.h"
#include "can_motor.h"
#include "cvector.h"
#include "referee.h"
#include "pub_sub.h"
// app layer
#include "robot_cmd.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

void APP_Layer_Init();
void APP_Layer_default_loop();
void APP_Log_Loop();
void APP_RobotCmd_Loop();

#endif