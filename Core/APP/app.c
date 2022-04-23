#include "app.h"

#include "robot_def.h"
// 功能模块
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
// 控制模块
#include "chassis_board_cmd.h"
#include "gimbal_board_cmd.h"

#ifdef GIMBAL_BOARD
// 重要模块
gimbal_board_cmd* cmd;
Gimbal* gimbal;
Shoot* shoot;
void APP_Layer_Init(){
    cmd=Gimbal_board_CMD_Create();
    gimbal = Gimbal_Create();
    shoot = Shoot_Create();
}

void APP_RobotCmd_Loop() {
    Gimbal_board_CMD_Update(cmd);
    Gimbal_Update(gimbal);
    Shoot_Update(shoot);
}
#endif

#ifdef CHASSIS_BOARD
Chassis* chassis;
void APP_Layer_Init(){
    chassis = Chassis_Create();
}

void APP_RobotCmd_Loop() {
    Robot_CMD_Update(robot);
    Chassis_Update(chassis);
}
#endif

// 打印输出等到ozone的窗口 用于测试项目
void APP_Log_Loop() {}


void APP_Layer_default_loop() {
    // if (chassis->imu->bias_init_success) {
    //     Buzzer_Update(internal_buzzer);
    // }
}
