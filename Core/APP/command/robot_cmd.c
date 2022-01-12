#include "robot_cmd.h"

// monitor处理函数
void core_module_lost(void* obj) {
    printf("core_module_lost!!!robot stopped for security.\n");
}


Robot* Robot_CMD_Create() {
    // 非模块内外设

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
    recv_config.notify_func = NULL;
    recv_config.lost_callback = core_module_lost;
    obj->board_com.send = CanSend_Create(&send_config);
    obj->board_com.recv = CanRecv_Create(&recv_config);

    return obj;
}

#ifdef CHASSIS_BOARD
void Robot_CMD_Update(Robot* robot) {
    // 板间通信同步
    memcpy(robot->board_com.goci_data,&(robot->board_com.recv->data_rx),sizeof(robot->board_com.goci_data));
    // 定义pub
    static Chassis_param chassis_param_pub;
    // 另一主控通知stop
    if (robot->board_com.goci_data->now_robot_mode == robot_stop) {
        robot->mode = robot_stop;
    }
    // 底盘重要外设丢失
    if (0)
    {
        robot->mode = robot_stop;
        robot->board_com.gico_data->if_chassis_board_module_lost = module_lost;
    }

    if (robot->mode = robot_stop) {
        chassis_param_pub.mode = chassis_stop;
    } else {
        memcpy(&(chassis_param_pub.target), &(robot->board_com.goci_data->chassis_target), sizeof(Chassis_param_speed_target));
    }

    static Publisher cmd_chassis;
}
#endif
#ifdef GIMBAL_BOARD
void Robot_CMD_Update(Robot* robot) {
}
#endif