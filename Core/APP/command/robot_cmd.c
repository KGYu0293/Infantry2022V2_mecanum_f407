#include "robot_cmd.h"


void core_module_lost() { printf("core_module_lost!!\n"); }
void motor_lost(void* motor) {
    printf_log("motor lost!\n");
    can_motor* now = (can_motor*)motor;
    now->monitor->reset(now->monitor);
}

void board_com_data_solve() {}

Robot* Robot_Create() {
    // 创建实例
    Robot* obj = (Robot*)malloc(sizeof(Robot));

    // 板间通信配置
    can_send_config send_config;
    can_recv_config recv_config;
    send_config.bsp_can_index = 1;
    recv_config.bsp_can_index = 1;
#ifdef CHASSIS_BOARD
    send_config.can_identifier = 0x203;
    recv_config.can_identifier = 0x204;
    send_config.data_len = sizeof(board_com_gico_data);
    recv_config.data_len = sizeof(board_com_goci_data);
#endif
#ifdef GIMBAL_BOARD
    send_config.can_identifier = 0x204;
    recv_config.can_identifier = 0x203;
    send_config.data_len = sizeof(board_com_goci_data);
    recv_config.data_len = sizeof(board_com_gico_data);
#endif
    recv_config.notify_func = board_com_data_solve;
    recv_config.lost_callback = core_module_lost;
    obj->board_com.send = CanSend_Create(&send_config);
    obj->board_com.recv = CanRecv_Create(&recv_config);

    // 模块创建
    // obj->chassis = Chassis_Create();

    return obj;
}

// APP层的函数，机器人命令层中枢，在app.h中声明并直接在rtos.c中执行
void APP_RobotCmd_Loop() {
    // 调用各部分update
}