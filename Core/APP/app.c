#include "app.h"

#include "chassis.h"
#include "robot_cmd.h"
#include "robot_param.h"

const char* chassis_cmd_topic = "cc";
const char* gimbal_cmd_topic = "gc";
const char* shoot_cmd_topic = "sc";
const char* chassis_upload_topic = "cu";
const char* gimbal_upload_topic = "gu";
const char* shoot_upload_topic = "su";

// 重要模块
Robot* robot;
Chassis* chassis;
Gimbal* gimbal;
Shoot* shoot;

// 共享外设
buzzer* internal_buzzer;

//此处定义外设的配置文件，也可分开文件配置
buzzer_config internal_buzzer_config;

void APP_Layer_Init() {
    // buzzer
#ifdef CHASSIS_BOARD
    uint32_t music_id = 1;
#endif
#ifdef GIMBAL_BOARD
    uint32_t music_id = 2;
#endif
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;
    internal_buzzer = Buzzer_Create(&internal_buzzer_config);

    robot = Robot_CMD_Create();
#ifdef CHASSIS_BOARD
    chassis = Chassis_Create();
#endif
#ifdef GIMBAL_BOARD
    gimbal = Gimbal_Create();
    shoot = Shoot_Create();
#endif
}

// 打印输出等到ozone的窗口 用于测试项目
void APP_Log_Loop() {}

#ifdef GIMBAL_BOARD
void APP_Layer_default_loop() {
    if (gimbal->imu->bias_init_success) {
        Buzzer_Update(internal_buzzer);
    }
}
// APP层的函数，机器人命令层中枢，在app.h中声明并直接在rtos.c中执行
void APP_RobotCmd_Loop() {
    // 调用各部分update
    // if (gimbal->imu->bias_init_success) {
    Robot_CMD_Update(robot);
    Gimbal_Update(gimbal);
    Shoot_Update(shoot);
    // }
}
#endif

#ifdef CHASSIS_BOARD
void APP_Layer_default_loop() {
    if (chassis->imu->bias_init_success) {
        Buzzer_Update(internal_buzzer);
    }
}
// APP层的函数，机器人命令层中枢，在app.h中声明并直接在rtos.c中执行
void APP_RobotCmd_Loop() {
    // 调用各部分update
    // if (chassis->imu->bias_init_success) {
    Robot_CMD_Update(robot);
    Chassis_Update(chassis);
    // }
}
#endif