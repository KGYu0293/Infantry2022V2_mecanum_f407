#include "app.h"

#include "chassis.h"
#include "robot_cmd.h"

// 重要模块
Robot* robot;
Chassis* chassis;
Gimbal* gimbal;
Shoot* shoot;

// 共享外设
buzzer* internal_buzzer;
referee* robot_referee;

//此处定义外设的配置文件，也可分开文件配置
buzzer_config internal_buzzer_config;

void APP_Layer_Init() {
    // buzzer
    uint32_t music_id = 1;
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;
    internal_buzzer = Buzzer_Create(&internal_buzzer_config);

    robot = Robot_CMD_Create();
    chassis = Chassis_Create();
    gimbal = Gimbal_Create();
    shoot = Shoot_Create();
    // fan = Fanlight_APP_Init();
}

void APP_Layer_default_loop() {
    if (gimbal->imu->bias_init_success) {
        Buzzer_Update(internal_buzzer);
    }
}

// 打印输出等到ozone的窗口 用于测试项目
void APP_Log_Loop() {
    if (gimbal->imu->bias_init_success) {
        // uint8_t buf[10] = "1234567812";
        // CanSend_Send(test_send, buf);
        // BSP_CAN_Send(1, 0x200, buf, 8);

        // static char fbufs[3][10];
        // Float2Str(fbufs[0], imu->data.euler_deg[0]);
        // Float2Str(fbufs[1], imu->data.euler_deg[1]);
        // Float2Str(fbufs[2], imu->data.euler_deg[2]);
        // printf_log("%s %s %s\n", fbufs[0], fbufs[1], fbufs[2]);

        // printf_log("test_log\n");
    }
}

// APP层的函数，机器人命令层中枢，在app.h中声明并直接在rtos.c中执行
void APP_RobotCmd_Loop() {
    // 调用各部分update
    Robot_CMD_Update(robot);
#ifdef CHASSIS_BOARD
    Chassis_Update(chassis);
#endif
#ifdef GIMBAL_BOARD
    Gimbal_Update(gimbal);
    Shoot_Update(shoot);
#endif
}