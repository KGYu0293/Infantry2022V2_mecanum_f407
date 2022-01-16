#include "robot_cmd.h"

// monitor处理函数
void core_module_lost(void* obj) { printf("core_module_lost!!!robot stopped for security.\n"); }

#ifdef GIMBAL_BOARD
Robot* Robot_CMD_Create() {
    // 创建实例
    Robot* obj = (Robot*)malloc(sizeof(Robot));

    // 板间通信配置
    can_send_config send_config;
    can_recv_config recv_config;
    send_config.bsp_can_index = 1;
    recv_config.bsp_can_index = 1;
    send_config.can_identifier = 0x004;
    recv_config.can_identifier = 0x003;
    send_config.data_len = sizeof(board_com_goci_data);
    recv_config.data_len = sizeof(board_com_gico_data);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = core_module_lost;
    obj->board_com.send = CanSend_Create(&send_config);
    obj->board_com.recv = CanRecv_Create(&recv_config);
    obj->board_com.goci_data = malloc(sizeof(board_com_goci_data));
    obj->board_com.gico_data = malloc(sizeof(board_com_gico_data));

    // 定义publisher和subscriber
    obj->gimbal_cmd_puber = register_pub(gimbal_cmd_topic);
    obj->gimbal_upload_suber = register_sub(gimbal_upload_topic, 1);
    obj->shoot_cmd_puber = register_pub(shoot_cmd_topic);

    // 外设初始化
    dt7_config remote_config;
    remote_config.bsp_uart_index = UART_REMOTE_PORT;
    remote_config.lost_callback = core_module_lost;
    obj->remote = dt7_Create(&remote_config);

    obj->mode = robot_stop;
    return obj;
}
void Robot_CMD_Update(Robot* robot) {
    // 判断机器人工作模式
    // 板间通信-收
    if ((robot->board_com.recv->monitor->count < 1)) {
        robot->mode = robot_stop;
    } else {
        memcpy(robot->board_com.gico_data, robot->board_com.recv->data_rx.data, sizeof(board_com_gico_data));
        // 重要外设判断
        if (robot->remote->monitor->count < 1) {
            robot->mode = robot_stop;
        }
        // 遥控器打到stop模式（右拨杆最下）
        else if (robot->remote->data.imput_mode == RC_Stop) {
            robot->mode = robot_stop;
        } else {
            robot->mode = robot_run;
        }
    }

    // 机器人控制
    if (robot->mode == robot_stop) {
        robot->board_com.goci_data->now_robot_mode = robot_stop;
        robot->board_com.goci_data->chassis_mode = chassis_stop;
        robot->gimbal_param.mode = gimbal_stop;
        robot->shoot_param.mode = shoot_stop;
    } else if (robot->mode == robot_run) {
        if (robot->remote->data.imput_mode == RC_Remote) {
            // gimbal
            robot->gimbal_param.mode = gimbal_run;
            robot->gimbal_param.yaw -= 0.05f * ((float)robot->remote->data.rc.ch2 - CHx_BIAS);
            robot->gimbal_param.pitch = -1.0f * ((float)robot->remote->data.rc.ch3 - CHx_BIAS);
            robot->gimbal_param.rotate_feedforward = 0;

            // chassis
            // 拨杆确定底盘模式与控制量
            robot->board_com.goci_data->chassis_target.vy = 16.0f * (float)(robot->remote->data.rc.ch1 - CHx_BIAS);
            robot->board_com.goci_data->chassis_target.vx = 16.0f * (float)(robot->remote->data.rc.ch0 - CHx_BIAS);
            if (robot->remote->data.rc.s1 == 1) {
                robot->board_com.goci_data->chassis_mode = chassis_rotate_run;
                robot->board_com.goci_data->chassis_target.vy *= 0.60f;
                robot->board_com.goci_data->chassis_target.vx *= 0.60f;
            } else {
                robot->board_com.goci_data->chassis_mode = chassis_run_follow_offset;
            }
            // 获取云台此时的offset
            publish_data gimbal_offset = robot->gimbal_upload_suber->getdata(robot->gimbal_upload_suber);
            if (gimbal_offset.len == -1) {
                robot->board_com.goci_data->chassis_target.offset_angle = 0;
            } else {
                short init_forward = 3152;
                short x = *(short*)(gimbal_offset.data);
                short tmp;
                if (x > init_forward && x <= 8192 - init_forward) {
                    tmp = x - init_forward;
                } else if (x > 8192 - init_forward) {
                    tmp = -8192 + x - init_forward;
                } else {
                    tmp = x - init_forward;
                }
                robot->board_com.goci_data->chassis_target.offset_angle = tmp / 8192.0 * 360.0;
            }

            // shoot
            robot->shoot_param.mode = shoot_run;
            if (robot->remote->data.rc.s1 == 2) {
                robot->shoot_param.mode = shoot_stop;
                if (robot->remote->data.rc.ch4 > CHx_BIAS + 400) robot->shoot_param.magazine_lid = magazine_on;
                if (robot->remote->data.rc.ch4 > CHx_BIAS - 400) robot->shoot_param.magazine_lid = magazine_off;
            } else {
                robot->shoot_param.mode = shoot_run;
                robot->shoot_param.shoot_command = continuous;
                robot->shoot_param.fire_rate = 0.01f * (float)(robot->remote->data.rc.ch4 - CHx_BIAS);
                robot->shoot_param.heat_limit_remain = robot->board_com.gico_data->shoot_referee_data.heat_limit_remain;
                robot->shoot_param.bullet_speed = robot->board_com.gico_data->shoot_referee_data.bullet_speed_max;
            }

        } else if (robot->remote->data.imput_mode == RC_MouseKey) {
            // chassis
            // shoot
            // offset
        }
    }
    // 发布变更
    publish_data gimbal_cmd;
    gimbal_cmd.data = (uint8_t*)&robot->gimbal_param;
    gimbal_cmd.len = sizeof(Gimbal_param);
    robot->gimbal_cmd_puber->publish(robot->gimbal_cmd_puber, gimbal_cmd);
    publish_data shoot_cmd;
    shoot_cmd.data = (uint8_t*)&robot->shoot_param;
    shoot_cmd.len = sizeof(Chassis_param);
    robot->shoot_cmd_puber->publish(robot->shoot_cmd_puber, shoot_cmd);

    // 板间通信-发
    if (robot->mode == robot_stop)
        robot->board_com.goci_data->now_robot_mode = robot_stop;
    else
        robot->board_com.goci_data->now_robot_mode = robot_run;
    // //debug
    // robot->board_com.goci_data->chassis_mode = chassis_stop;
    CanSend_Send(robot->board_com.send, (uint8_t*)robot->board_com.goci_data);
}
#endif

#ifdef CHASSIS_BOARD
Robot* Robot_CMD_Create() {
    // 创建实例
    Robot* obj = (Robot*)malloc(sizeof(Robot));

    // 板间通信配置
    can_send_config send_config;
    can_recv_config recv_config;
    send_config.bsp_can_index = 1;
    recv_config.bsp_can_index = 1;
    send_config.can_identifier = 0x003;
    recv_config.can_identifier = 0x004;
    send_config.data_len = sizeof(board_com_gico_data);
    recv_config.data_len = sizeof(board_com_goci_data);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = core_module_lost;
    obj->board_com.send = CanSend_Create(&send_config);
    obj->board_com.recv = CanRecv_Create(&recv_config);
    obj->board_com.goci_data = malloc(sizeof(board_com_goci_data));
    obj->board_com.gico_data = malloc(sizeof(board_com_gico_data));

    referee_config referee_config;
    referee_config.bsp_uart_index = UART_REFEREE_PORT;
    referee_config.lost_callback = NULL;
    // obj->referee = referee_Create(&referee_config);

    // 定义publisher和subscriber
    obj->chassis_cmd_puber = register_pub(chassis_cmd_topic);

    obj->chassis_upload_sub = register_sub(chassis_upload_topic, 1);
    obj->mode = robot_stop;
    return obj;
}

void Robot_CMD_Update(Robot* robot) {
    // 板间通信-收
    if (robot->board_com.recv->monitor->count < 1) {
        robot->mode = robot_stop;
    } else {
        memcpy(robot->board_com.goci_data, robot->board_com.recv->data_rx.data, sizeof(board_com_goci_data));
        // 底盘重要外设丢失
        if (0) {
            robot->mode = robot_stop;
            robot->board_com.gico_data->if_chassis_board_module_lost = module_lost;
        } else {
            robot->board_com.gico_data->if_chassis_board_module_lost = module_working;
            // 主板stop指令
            if (robot->board_com.goci_data->now_robot_mode == robot_stop) {
                robot->mode = robot_stop;
            } else {
                robot->mode = robot_run;
            }
        }
    }

    // 底盘控制
    if (robot->mode == robot_stop) {
        robot->chassis_param.mode = chassis_stop;
    } else {
        robot->chassis_param.mode = robot->board_com.goci_data->chassis_mode;
        memcpy(&(robot->chassis_param.target), &(robot->board_com.goci_data->chassis_target), sizeof(Chassis_param_speed_target));
    }
    // robot->chassis_param.power.power_buffer = robot->referee->rx_data.power_heat.chassis_power_buffer;
    // robot->chassis_param.power.power_now = robot->referee->rx_data.power_heat.chassis_power;
    // robot->chassis_param.power.power_limit = robot->referee->rx_data.game_robot_state.chassis_power_limit;
    robot->chassis_param.power.power_buffer = 0;
    robot->chassis_param.power.power_now = 30;
    robot->chassis_param.power.power_limit = 50;

    // 发布变更
    publish_data chassis_cmd;
    chassis_cmd.data = (uint8_t*)&robot->chassis_param;
    chassis_cmd.len = sizeof(Chassis_param);
    robot->chassis_cmd_puber->publish(robot->chassis_cmd_puber, chassis_cmd);

    // 获取底盘imu数据
    publish_data chassis_imu_data = robot->chassis_upload_sub->getdata(robot->chassis_upload_sub);
    if (chassis_imu_data.len == -1)
        robot->board_com.gico_data->gyro_yaw = 0;
    else
        // memcpy(&(robot->board_com.gico_data)->chassis_imu_data, chassis_imu_data.data, sizeof(imu_data));
        robot->board_com.gico_data->gyro_yaw = ((imu_data*)chassis_imu_data.data)->gyro[2];
    // 发送信息底盘->云台
    // robot->board_com.gico_data->shoot_referee_data.bullet_speed_max = robot->referee->rx_data.game_robot_state.shooter_id1_17mm_speed_limit;
    // robot->board_com.gico_data->shoot_referee_data.heat_limit_remain =
    //     robot->referee->rx_data.game_robot_state.shooter_id1_17mm_cooling_limit - robot->referee->rx_data.power_heat.shooter_id1_17mm_cooling_heat;
    robot->board_com.gico_data->shoot_referee_data.bullet_speed_max = 15;
    robot->board_com.gico_data->shoot_referee_data.heat_limit_remain = 30;
    CanSend_Send(robot->board_com.send, (uint8_t*)robot->board_com.gico_data);
}
#endif