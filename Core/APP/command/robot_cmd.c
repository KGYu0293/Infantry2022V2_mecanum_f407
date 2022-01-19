#include "robot_cmd.h"

// monitor处理函数
void core_module_lost(void* obj) { printf_log("core_module_lost!!!robot stopped for security.\n"); }
void pc_lost(void* obj) { printf_log("pc lost!\n"); }

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

    // 小电脑通信配置
    canpc_config pc_config;
    pc_config.bsp_can_index = 1;
    pc_config.recv_identifer = 0x001;
    pc_config.send_identifer = 0x002;
    pc_config.lost_callback = pc_lost;
    obj->pc = CanPC_Create(&pc_config);

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
    obj->ready = 0;
    return obj;
}

void Robot_CMD_Update(Robot* robot) {
    // 判断机器人工作模式
    //初始化为RUN
    robot->mode = robot_run;
    Gimbal_uplode_data* gimbal_upload_data;

    // 板间通信-收
    if ((robot->board_com.recv->monitor->count < 1)) {
        robot->mode = robot_stop;
    }
    // 判断云台IMU是否上线
    publish_data gimbal_data_fdb = robot->gimbal_upload_suber->getdata(robot->gimbal_upload_suber);
    if (gimbal_data_fdb.len == -1) {
        robot->board_com.goci_data->chassis_target.offset_angle = 0;
        robot->mode = robot_stop;
    } else {
        gimbal_upload_data = (Gimbal_uplode_data*)gimbal_data_fdb.data;
        if (gimbal_upload_data->gimbal_module_status == module_lost) robot->mode = robot_stop;
    }

    // 除了遥控器之外都已经上线
    if (robot->mode == robot_run) {
        robot->ready = 1;
    }

    // 遥控器判断
    if (robot->remote->monitor->count < 1) {
        robot->mode = robot_stop;
    } else {
        if (robot->remote->data.imput_mode == RC_Stop) {
            robot->mode = robot_stop;
        }
    }

    // 机器人控制
    if (robot->mode == robot_stop) {
        robot->board_com.goci_data->now_robot_mode = robot_stop;
        robot->board_com.goci_data->chassis_mode = chassis_stop;
        robot->gimbal_param.mode = gimbal_stop;
        robot->shoot_param.mode = shoot_stop;
    } else if (robot->mode == robot_run) {
        // 小电脑通信
        CanPC_Send(robot->pc, (canpc_send*)gimbal_upload_data->gimbal_imu_euler);
        // 获取云台offset
        short init_forward = 3152;  // 云台朝向底盘正前时云台yaw编码器值
        short x = gimbal_upload_data->yaw_encorder;
        short tmp;
        if (x > init_forward && x <= 8192 - init_forward) {
            tmp = x - init_forward;
        } else if (x > 8192 - init_forward) {
            tmp = -8192 + x - init_forward;
        } else {
            tmp = x - init_forward;
        }
        robot->board_com.goci_data->chassis_target.offset_angle = tmp / 8192.0 * 360.0;

        // 遥控器控制模式
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
        }

        // 键鼠控制模式
        else if (robot->remote->data.imput_mode == RC_MouseKey) {
            // robot_state
            static enum { chassis_follow_gimbal, gimbal_follow_chassis, independent } chassis_gimbal_follow_mode = chassis_follow_gimbal;
            static enum { auto_aim_off, auto_aim_on, auto_aim_AtkBuff } auto_aim_mode;

            // 按一下r:小陀螺
            if (robot->remote->data.key_single_press_cnt.r != robot->remote->last_data.key_single_press_cnt.r) {
                if (robot->board_com.goci_data->chassis_mode != chassis_rotate_run) {
                    robot->board_com.goci_data->chassis_mode = chassis_rotate_run;
                    chassis_gimbal_follow_mode = independent;
                } else {
                    chassis_gimbal_follow_mode = chassis_follow_gimbal;
                }
            }
            // x:跟随底盘(飞坡)
            if (robot->remote->data.key_single_press_cnt.x != robot->remote->last_data.key_single_press_cnt.x) {
                if (robot->board_com.goci_data->chassis_mode != chassis_run) {
                    robot->board_com.goci_data->chassis_mode = chassis_run;
                    chassis_gimbal_follow_mode = gimbal_follow_chassis;
                } else {
                    chassis_gimbal_follow_mode = chassis_follow_gimbal;
                }
            }
            // v:云台底盘独立
            if (robot->remote->data.key_single_press_cnt.v != robot->remote->last_data.key_single_press_cnt.v) {
                chassis_gimbal_follow_mode = independent;
            }
            // z:爬坡 （）待添加

            // f:自瞄
            if (robot->remote->data.key_single_press_cnt.f != robot->remote->last_data.key_single_press_cnt.f) {
                if (auto_aim_mode != auto_aim_on)
                    auto_aim_mode = auto_aim_on;
                else
                    auto_aim_mode = auto_aim_off;
            }
            // g:小符  // b:大符
            if (robot->remote->data.key_single_press_cnt.g != robot->remote->last_data.key_single_press_cnt.g) {
                if (auto_aim_mode != auto_aim_on)
                    auto_aim_mode = auto_aim_on;
                else
                    auto_aim_mode = auto_aim_off;
            }

            if ((auto_aim_mode != auto_aim_off) && (chassis_gimbal_follow_mode == gimbal_follow_chassis)) {
                chassis_gimbal_follow_mode = chassis_follow_gimbal;
            }

            // robot_param
            // 平移
            if (robot->remote->data.key_down.w) robot->board_com.goci_data->chassis_target.vy = 8000;
            if (robot->remote->data.key_down.s) robot->board_com.goci_data->chassis_target.vy = -8000;
            if (robot->remote->data.key_down.d) robot->board_com.goci_data->chassis_target.vx = 8000;
            if (robot->remote->data.key_down.a) robot->board_com.goci_data->chassis_target.vx = -8000;
            // 按住ctrl减速
            if (robot->remote->data.key_down.ctrl) {
                robot->board_com.goci_data->chassis_target.vx /= 3;
                robot->board_com.goci_data->chassis_target.vy /= 3;
            }
            // shift加速
            if (robot->remote->data.key_down.shift) {
                robot->board_com.goci_data->chassis_target.vx *= 3;
                robot->board_com.goci_data->chassis_target.vy *= 3;
            }
            // rotate/gimbal
            switch (chassis_gimbal_follow_mode) {
                case gimbal_follow_chassis:
                    robot->board_com.goci_data->chassis_target.rotate -= 0.5f * (0.7f * (robot->remote->data.mouse.x) + 0.3f * (robot->remote->last_data.mouse.x));
                    if (robot->remote->data.key_down.q) robot->board_com.goci_data->chassis_target.rotate = -90;
                    if (robot->remote->data.key_down.e) robot->board_com.goci_data->chassis_target.rotate = 90;
                    robot->board_com.goci_data->chassis_mode = chassis_run;
                    robot->gimbal_param.mode = gimbal_middle;
                    break;
                case chassis_follow_gimbal:
                    robot->gimbal_param.yaw -= 0.5f * (0.7f * (robot->remote->data.mouse.x) + 0.3f * (robot->remote->last_data.mouse.x));
                    if (robot->remote->data.key_down.q) robot->gimbal_param.yaw -= 15;
                    if (robot->remote->data.key_down.e) robot->gimbal_param.yaw += 15;
                    robot->board_com.goci_data->chassis_mode = chassis_run_follow_offset;
                    break;
                case independent:
                    robot->gimbal_param.yaw -= 0.5f * (0.7f * (robot->remote->data.mouse.x) + 0.3f * (robot->remote->last_data.mouse.x));
                    if (robot->remote->data.key_down.q) robot->board_com.goci_data->chassis_target.rotate = -90;
                    if (robot->remote->data.key_down.e) robot->board_com.goci_data->chassis_target.rotate = 90;
                    break;
            }
            // gimbal_auto_aim

            // shoot
            // 按C开关弹仓
            if (robot->remote->data.key_single_press_cnt.c % 2)
                robot->shoot_param.magazine_lid = magazine_off;
            else
                robot->shoot_param.magazine_lid = magazine_on;
        }
    }
    // 发布变更
    publish_data gimbal_cmd;
    gimbal_cmd.data = (uint8_t*)&robot->gimbal_param;
    gimbal_cmd.len = sizeof(Gimbal_param);
    robot->gimbal_cmd_puber->publish(robot->gimbal_cmd_puber, gimbal_cmd);
    publish_data shoot_cmd;
    shoot_cmd.data = (uint8_t*)&robot->shoot_param;
    shoot_cmd.len = sizeof(Shoot_param);
    robot->shoot_cmd_puber->publish(robot->shoot_cmd_puber, shoot_cmd);

    // 板间通信-发
    if (robot->mode == robot_stop)
        robot->board_com.goci_data->now_robot_mode = robot_stop;
    else
        robot->board_com.goci_data->now_robot_mode = robot_run;
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
            robot->board_com.gico_data->chassis_board_status = module_lost;
        } else {
            robot->board_com.gico_data->chassis_board_status = module_working;
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