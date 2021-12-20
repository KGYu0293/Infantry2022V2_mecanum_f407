#ifndef _APP_H
#define _APP_H
#include "BMI088.h"
#include "buzzer.h"
#include "can_send.h"
#include "can_recv.h"
#include "can_pc.h"
#include "can_motor.h"
#include "DT7_DR16.h"
#include "fanlight.h"


// #TODO to add other motors

void APP_Layer_Init();
void APP_Layer_default_loop();
void APP_Log_Loop();
void APP_FanLight_Loop();
#endif