#include "chassis_board_cmd.h"

// monitor处理函数
void chassis_core_module_lost(void* obj) {
    // 暂时仅调试
    printf_log("chassis_core_module_lost!!!robot stopped for security.\n");
}

chassis_board_cmd* Chassis_board_CMD_Create() {
    // 创建实例
    chassis_board_cmd* obj = (chassis_board_cmd*)malloc(sizeof(chassis_board_cmd));

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
    recv_config.lost_callback = chassis_core_module_lost;
    obj->board_com.send = CanSend_Create(&send_config);
    obj->board_com.recv = CanRecv_Create(&recv_config);
    obj->board_com.goci_data = malloc(sizeof(board_com_goci_data));
    obj->board_com.gico_data = malloc(sizeof(board_com_gico_data));

    buzzer_config internal_buzzer_config;
    uint32_t music_id = 1;
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;
    obj->internal_buzzer = Buzzer_Create(&internal_buzzer_config);

    referee_config referee_config;
    referee_config.bsp_uart_index = UART_REFEREE_PORT;
    referee_config.lost_callback = NULL;
    // obj->referee = referee_Create(&referee_config);

    // 定义publisher和subscriber
    obj->chassis_cmd_puber = register_pub(chassis_cmd_topic);
    obj->chassis_upload_sub = register_sub(chassis_upload_topic, 1);
    
    // memset 0
    obj->mode = robot_stop;
    
    return obj;
}

void Chassis_board_CMD_Update(chassis_board_cmd* obj) {
    // 初始化为RUN
    obj->mode = robot_run;
    // 判断板间通信在线
    if (obj->board_com.recv->monitor->count < 1) {
        obj->mode = robot_stop;
    } else {
        memcpy(obj->board_com.goci_data, obj->board_com.recv->data_rx.data, sizeof(board_com_goci_data));
    }
    // 底盘重要外设丢失
    if (0) {
        obj->mode = robot_stop;
        obj->board_com.gico_data->chassis_board_status = module_lost;
    } else {
        obj->board_com.gico_data->chassis_board_status = module_working;
    }
    // 主板stop指令
    if (obj->board_com.goci_data->now_robot_mode == robot_stop) {
        obj->mode = robot_stop;
    } else {
        obj->mode = robot_run;
    }

    // 底盘控制
    if (obj->mode == robot_stop) {
        obj->chassis_param.mode = chassis_stop;
    } else {
        obj->chassis_param.mode = obj->board_com.goci_data->chassis_mode;
        memcpy(&(obj->chassis_param.target), &(obj->board_com.goci_data->chassis_target), sizeof(Chassis_param_speed_target));
    }
    // robot->chassis_param.power.power_buffer = robot->referee->rx_data.power_heat.chassis_power_buffer;
    // robot->chassis_param.power.power_now = robot->referee->rx_data.power_heat.chassis_power;
    // robot->chassis_param.power.power_limit = robot->referee->rx_data.game_robot_state.chassis_power_limit;
    obj->chassis_param.power.power_buffer = 0;
    obj->chassis_param.power.power_now = 30;
    obj->chassis_param.power.power_limit = 50;
    obj->chassis_param.power.if_supercap_on = obj->board_com.goci_data->if_supercap_on;

    // 发布变更
    publish_data chassis_cmd;
    chassis_cmd.data = (uint8_t*)&obj->chassis_param;
    chassis_cmd.len = sizeof(Chassis_param);
    obj->chassis_cmd_puber->publish(obj->chassis_cmd_puber, chassis_cmd);

    // 获取底盘imu数据
    publish_data chassis_imu_data = obj->chassis_upload_sub->getdata(obj->chassis_upload_sub);
    if (chassis_imu_data.len == -1)
        obj->board_com.gico_data->gyro_yaw = 0;
    else
        obj->board_com.gico_data->gyro_yaw = ((imu_data*)chassis_imu_data.data)->gyro[2];
    // 发送信息底盘->云台
    // obj->board_com.gico_data->shoot_referee_data.bullet_speed_max = obj->referee->rx_data.game_robot_state.shooter_id1_17mm_speed_limit;
    // obj->board_com.gico_data->shoot_referee_data.heat_limit_remain =
    //     obj->referee->rx_data.game_robot_state.shooter_id1_17mm_cooling_limit - obj->referee->rx_data.power_heat.shooter_id1_17mm_cooling_heat;
    obj->board_com.gico_data->shoot_referee_data.bullet_speed_max = 15;
    obj->board_com.gico_data->shoot_referee_data.heat_limit_remain = 30;
    CanSend_Send(obj->board_com.send, (uint8_t*)obj->board_com.gico_data);
}