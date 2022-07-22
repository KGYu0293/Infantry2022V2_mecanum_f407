#include "chassis_board_cmd.h"

#include "bsp.h"

// monitor处理函数
void chassis_core_module_lost(void* obj) {
    // 暂时仅调试
    printf_log("chassis_core_module_lost!!!robot stopped for security.\n");
}

chassis_board_cmd* Chassis_board_CMD_Create() {
    // 创建实例
    chassis_board_cmd* obj = (chassis_board_cmd*)malloc(sizeof(chassis_board_cmd));
    memset(obj, 0, sizeof(chassis_board_cmd));

    // 板间通信配置
    can_send_config send_config;
    can_recv_config recv_config;
    send_config.bsp_can_index = 1;
    recv_config.bsp_can_index = 1;
    send_config.can_identifier = 0x003;
    recv_config.can_identifier = 0x004;
    send_config.data_len = sizeof(Chassis_board_send_data);
    recv_config.data_len = sizeof(Gimbal_board_send_data);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = chassis_core_module_lost;
    obj->send = CanSend_Create(&send_config);
    obj->recv = CanRecv_Create(&recv_config);
    obj->recv_data = (Gimbal_board_send_data*)obj->recv->data_rx.data;

    //蜂鸣器配置
    buzzer_config internal_buzzer_config;
    uint32_t music_id = 5;
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;
    obj->internal_buzzer = Buzzer_Create(&internal_buzzer_config);

    //裁判系统
    referee_config referee_config;
    referee_config.bsp_uart_index = UART_REFEREE_PORT;
    referee_config.lost_callback = NULL;
    obj->referee = referee_Create(&referee_config);
    obj->ui = NULL;  //初始化为NULL

    Indicator_led_config led_config;
    led_config.bsp_gpio_led1_index = GPIO_LED1;
    led_config.bsp_gpio_led2_index = GPIO_LED2;
    led_config.bsp_gpio_led3_index = GPIO_LED3;
    obj->indicator_led = indicator_led_Create(&led_config);

    // 定义publisher和subscriber
    obj->chassis_cmd_puber = register_pub("cmd_chassis");
    obj->chassis_upload_sub = register_sub("upload_chassis", 1);

    // memset 0
    memset(&(obj->send_data), 0, sizeof(Chassis_board_send_data));
    memset(&(obj->chassis_control), 0, sizeof(Cmd_chassis));
    obj->chassis_upload_data = NULL;
    obj->robot_ready = 0;
    return obj;
}

void Chassis_board_CMD_Update(chassis_board_cmd* obj) {
    // 初始化为RUN
    obj->mode = robot_run;

    // 底盘板掉线检测与处理
    // 判断板间通信在线
    if (obj->recv->monitor->count < 1) {
        obj->mode = robot_stop;
    }

    // 接收底盘回传信息，判断底盘IMU在线且初始化完成
    publish_data chassis_upload = obj->chassis_upload_sub->getdata(obj->chassis_upload_sub);
    if (chassis_upload.len == -1) {
        obj->send_data.gyro_yaw = 0;
        obj->mode = robot_stop;
    } else {
        // obj->send_data.gyro_yaw = ((imu_data*)chassis_upload_data.data)->gyro[2];
        obj->chassis_upload_data = (Upload_chassis*)chassis_upload.data;
        obj->send_data.gyro_yaw = obj->chassis_upload_data->chassis_imu->gyro[2];
        obj->send_data.chassis_supercap_percent = (uint8_t)obj->chassis_upload_data->chassis_supercap_percent;
        // 底盘模块掉线
        if (obj->chassis_upload_data->chassis_status == module_lost) {
            obj->mode = robot_stop;
        }
    }

    // 底盘重要外设丢失
    if (obj->chassis_upload_data == NULL) {
        obj->mode = robot_stop;
    }

    // 裁判系统在线
    if (obj->referee->monitor->count > 0 && obj->referee->robot_status_received > 0) {
        //第一次收到时候，初始化UI
        if (obj->referee->robot_status_received == 1) {
            robot_ui_config ui_config;
            ui_config.referee = obj->referee;
            // ui_config.robot_id = obj->referee->rx_data.game_robot_state.robot_id;
            obj->ui = Create_Robot_UI(&ui_config);
            obj->referee->robot_status_received = 2;
        }
    } else {
        // obj->mode = robot_stop;
        obj->referee->rx_data.game_robot_state.shooter_id1_17mm_speed_limit = 15;
        obj->referee->rx_data.game_robot_state.shooter_id1_17mm_cooling_limit = 100;
        obj->referee->rx_data.power_heat.shooter_id1_17mm_cooling_heat = 0;
    }

    // 判断除了云台板stop之外，都已经上线，说明底盘板初始化完成，进入ready状态
    if (obj->mode == robot_run) {
        obj->send_data.chassis_board_status = module_working;
        if (!obj->robot_ready) {
            obj->robot_ready = 1;
            // 播放音乐
            Buzzer_Start(obj->internal_buzzer);
        }
    } else {
        obj->send_data.chassis_board_status = module_lost;
    }

    // 主板stop指令
    if (obj->recv_data->now_robot_mode == robot_stop) {
        obj->mode = robot_stop;
    } else {
        obj->mode = robot_run;
    }

    obj->chassis_control.power.dispatch_mode = obj->recv_data->chassis_dispatch_mode;
    obj->chassis_control.power.power_buffer = obj->referee->rx_data.power_heat.chassis_power_buffer;
    obj->chassis_control.power.power_now = obj->referee->rx_data.power_heat.chassis_power;
    obj->chassis_control.power.power_limit = obj->referee->rx_data.game_robot_state.chassis_power_limit;
    // 底盘控制指令
    if (obj->mode == robot_stop) {
        obj->chassis_control.mode = chassis_stop;
    } else {
        obj->chassis_control.mode = obj->recv_data->chassis_mode;
        obj->chassis_control.target = obj->recv_data->chassis_target;
        // obj->chassis_control.power.if_consume_supercap = obj->recv_data->if_consume_supercap;
    }

    // 发布指令
    publish_data chassis_cmd;
    chassis_cmd.data = (uint8_t*)&obj->chassis_control;
    chassis_cmd.len = sizeof(Cmd_chassis);
    obj->chassis_cmd_puber->publish(obj->chassis_cmd_puber, chassis_cmd);

    // 板间通信
    obj->send_data.shoot_referee_data.bullet_speed_max = obj->referee->rx_data.game_robot_state.shooter_id1_17mm_speed_limit;
    if(obj->referee->rx_data.game_robot_state.shooter_id1_17mm_cooling_limit == 65535) obj->send_data.shoot_referee_data.heat_limit_remain = 1000;
    else obj->send_data.shoot_referee_data.heat_limit_remain = obj->referee->rx_data.game_robot_state.shooter_id1_17mm_cooling_limit - obj->referee->rx_data.power_heat.shooter_id1_17mm_cooling_heat;
    obj->send_data.robot_id = obj->referee->rx_data.game_robot_state.robot_id;
    obj->send_data.shoot_cnt = obj->referee->shoot_cnt;
    CanSend_Send(obj->send, (uint8_t*)&(obj->send_data));

    // 更新UI信息
    if (obj->ui != NULL) {
        if (obj->chassis_upload_data != NULL) {
            obj->ui->data.cap_percent = obj->chassis_upload_data->chassis_supercap_percent;
            obj->ui->data.bat_voltage = obj->chassis_upload_data->chassis_battery_voltage;
            obj->ui->data.fri_mode = obj->recv_data->fri_mode;
            obj->ui->data.mag_mode = obj->recv_data->mag_mode;
            obj->ui->data.gimbal_mode = obj->recv_data->gimbal_mode;
            obj->ui->data.autoaim_mode = obj->recv_data->autoaim_mode;
            obj->ui->data.chassis_mode = obj->recv_data->chassis_mode;
            obj->ui->data.pc_online = obj->recv_data->pc_online;
            obj->ui->data.vision_has_taget = obj->recv_data->vision_has_target;
            obj->ui->data.power_mode = obj->recv_data->chassis_dispatch_mode;
            obj->ui->data.bullet_speed = obj->referee->rx_data.game_robot_state.shooter_id1_17mm_speed_limit;
            obj->ui->data.angle = obj->recv_data->chassis_target.offset_angle * 3.14159 / 180;
        }
        Robot_UI_update(obj->ui);
    }
}