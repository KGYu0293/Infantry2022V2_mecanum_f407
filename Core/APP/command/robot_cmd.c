#include "robot_cmd.h"

// monitor处理函数
void core_module_lost(void* obj) { printf("core_module_lost!!!robot stopped for security.\n"); }

Robot* Robot_CMD_Create() {
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

// 定义publisher和subscriber
#ifdef CHASSIS_BOARD
obj->chassis_cmd_puber = register_pub(chassis_cmd_topic);
#endif
#ifdef GIMBAL_BOARD
obj->gimbal_cmd_puber = register_pub(gimbal_cmd_topic);
obj->gimbal_upload_suber = register_sub(gimbal_uplode_topic,sizeof(Gimbal_uplode_data));
obj->shoot_puber = register_pub(shoot_cmd_topic);
#endif
    return obj;
}

#ifdef CHASSIS_BOARD
void Robot_CMD_Update(Robot* robot) {
    // 板间通信同步
    memcpy(robot->board_com.goci_data, &(robot->board_com.recv->data_rx), sizeof(robot->board_com.goci_data));


    // 另一主控通知stop
    if (robot->board_com.goci_data->now_robot_mode == robot_stop) {
        robot->mode = robot_stop;
    }
    // 底盘重要外设丢失
    if (0) {
        robot->mode = robot_stop;
        robot->board_com.gico_data->if_chassis_board_module_lost = module_lost;
    }

    if (robot->mode = robot_stop) {
        robot->chassis_param.mode = chassis_stop;
    } else {
        memcpy(&(robot->chassis_param.target), &(robot->board_com.goci_data->chassis_target), sizeof(Chassis_param_speed_target));
    }

    publish_data chassis_data;
    chassis_data.data = &robot->chassis_param;
    chassis_data.len = sizeof(Chassis_param);

    robot->chassis_cmd_puber->publish(robot->chassis_cmd_puber,chassis_data);
}
#endif
#ifdef GIMBAL_BOARD
void Robot_CMD_Update(Robot* robot) {}
#endif