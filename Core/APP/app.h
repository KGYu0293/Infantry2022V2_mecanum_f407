#ifndef _APP_H
#define _APP_H

#include "bsp_log.h"

void APP_Layer_Init();
void APP_Layer_default_loop();
// APP层的函数，输出调试信息
void APP_Log_Loop();
// APP层的函数，机器人命令层中枢，在app.h中声明并直接在rtos.c中执行
void APP_RobotCmd_Loop();

#endif