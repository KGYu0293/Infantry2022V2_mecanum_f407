#include "gimbal_board_cmd.h"

#define INIT_FORWARD 3152  // 云台朝向底盘正前时云台yaw编码器值
// monitor处理函数
void gimbal_core_module_lost(void* obj) { printf_log("gimbal_core_module_lost!!!robot stopped for security.\n"); }
void pc_lost(void* obj) { printf_log("pc lost!\n"); }
// cmd的private函数
void stop_mode_update(gimbal_board_cmd* obj);       //机器人停止模式更新函数
void remote_mode_update(gimbal_board_cmd* obj);     //机器人遥控器模式更新函数
void mouse_key_mode_update(gimbal_board_cmd* obj);  //机器人键鼠模式更新函数
void send_cmd_and_data(gimbal_board_cmd* obj);      //发布指令和板间通信
// 其他功能函数
float get_offset_angle(short init_forward, short now_encoder);  // 获取云台朝向与底盘正前的夹角

gimbal_board_cmd* Gimbal_board_CMD_Create() {
    // 创建实例
    gimbal_board_cmd* obj = (gimbal_board_cmd*)malloc(sizeof(gimbal_board_cmd));

    // 板间通信配置
    can_send_config send_config;
    can_recv_config recv_config;
    send_config.bsp_can_index = 1;
    recv_config.bsp_can_index = 1;
    send_config.can_identifier = 0x004;
    recv_config.can_identifier = 0x003;
    send_config.data_len = sizeof(Gimbal_board_send_data);
    recv_config.data_len = sizeof(Chassis_board_send_data);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = gimbal_core_module_lost;
    obj->send = CanSend_Create(&send_config);
    obj->recv = CanRecv_Create(&recv_config);
    obj->recv_data = obj->recv->data_rx.data;

    // 小电脑通信配置
    canpc_config pc_config;
    pc_config.bsp_can_index = 1;
    pc_config.recv_identifer = 0x001;
    pc_config.send_identifer = 0x002;
    pc_config.lost_callback = pc_lost;
    obj->pc = CanPC_Create(&pc_config);

    // 定义publisher和subscriber
    obj->gimbal_cmd_puber = register_pub("cmd_gimbal");
    obj->gimbal_upload_suber = register_sub("upload_gimbal", 1);
    obj->shoot_cmd_puber = register_pub("cmd_shoot");

    // 外设初始化
    buzzer_config internal_buzzer_config;
    uint32_t music_id = 2;
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;
    obj->internal_buzzer = Buzzer_Create(&internal_buzzer_config);

    dt7_config remote_config;
    remote_config.bsp_uart_index = UART_REMOTE_PORT;
    remote_config.lost_callback = gimbal_core_module_lost;
    obj->remote = dt7_Create(&remote_config);

    // memset 0
    obj->mode = robot_stop;
    obj->send_data.if_supercap_on = 0;  //电容默认关闭
    obj->ready = 0;
    return obj;
}

void Gimbal_board_CMD_Update(gimbal_board_cmd* obj) {
    // 判断机器人工作模式
    //初始化为RUN
    obj->mode = robot_run;
    Upload_gimbal* gimbal_upload_data;
    // 小电脑通信结构体
    canpc_send pc_send_data;
    memset(&pc_send_data, 0, sizeof(pc_send_data));

    // 板间通信-收
    if ((obj->recv->monitor->count < 1)) {
        obj->mode = robot_stop;
    }
    // 判断云台IMU是否上线
    publish_data gimbal_data_fdb = obj->gimbal_upload_suber->getdata(obj->gimbal_upload_suber);
    if (gimbal_data_fdb.len == -1) {
        obj->send_data.chassis_target.offset_angle = 0;
        obj->mode = robot_stop;
    } else {
        gimbal_upload_data = (Upload_gimbal*)gimbal_data_fdb.data;
        pc_send_data.euler[0] = gimbal_upload_data->gimbal_imu->euler[0];
        pc_send_data.euler[1] = gimbal_upload_data->gimbal_imu->euler[1];
        pc_send_data.euler[2] = gimbal_upload_data->gimbal_imu->euler[2];
        if (gimbal_upload_data->gimbal_status == module_lost) obj->mode = robot_stop;
    }

    // 除了遥控器之外都已经上线
    if (obj->mode == robot_run) {
        obj->ready = 1;
        // buzzer
    }

    // 遥控器判断
    if (obj->remote->monitor->count < 1) {
        obj->mode = robot_stop;
    } else {
        if (obj->remote->data.input_mode == RC_Stop) {
            obj->mode = robot_stop;
        }
    }
    // 机器人控制
    if (obj->mode == robot_stop) {
        stop_mode_update(obj);
    } else if (obj->mode == robot_run) {
        // 获取云台offset
        obj->send_data.chassis_target.offset_angle = get_offset_angle(INIT_FORWARD, *gimbal_upload_data->yaw_encorder);
        // 自瞄关
        pc_send_data.auto_mode_flag = 0;
        // 遥控器控制模式
        if (obj->remote->data.input_mode == RC_Remote) {
            remote_mode_update(obj);
        }
        // 键鼠控制模式
        else if (obj->remote->data.input_mode == RC_MouseKey) {
            // robot_state
            static enum { chassis_follow_gimbal, gimbal_follow_chassis, independent } chassis_gimbal_follow_mode = chassis_follow_gimbal;
            static enum { auto_aim_off, auto_aim_on, auto_aim_AtkBuff_small, auto_aim_AtkBuff_big } auto_aim_mode;

            // 按一下r:小陀螺
            if (obj->remote->data.key_single_press_cnt.r != obj->remote->last_data.key_single_press_cnt.r) {
                if (obj->send_data.chassis_mode != chassis_rotate_run) {
                    obj->send_data.chassis_mode = chassis_rotate_run;
                    chassis_gimbal_follow_mode = independent;
                } else {
                    chassis_gimbal_follow_mode = chassis_follow_gimbal;
                }
            }
            // x:跟随底盘(飞坡)
            if (obj->remote->data.key_single_press_cnt.x != obj->remote->last_data.key_single_press_cnt.x) {
                if (obj->send_data.chassis_mode != chassis_run) {
                    obj->send_data.chassis_mode = chassis_run;
                    chassis_gimbal_follow_mode = gimbal_follow_chassis;
                } else {
                    chassis_gimbal_follow_mode = chassis_follow_gimbal;
                }
            }
            // v:云台底盘独立
            if (obj->remote->data.key_single_press_cnt.v != obj->remote->last_data.key_single_press_cnt.v) {
                chassis_gimbal_follow_mode = independent;
            }
            // z:爬坡 （）待添加

            // f:自瞄
            if (obj->remote->data.key_single_press_cnt.f != obj->remote->last_data.key_single_press_cnt.f) {
                if (auto_aim_mode != auto_aim_on)
                    auto_aim_mode = auto_aim_on;
                else
                    auto_aim_mode = auto_aim_off;
            }
            // g:小符
            if (obj->remote->data.key_single_press_cnt.g != obj->remote->last_data.key_single_press_cnt.g) {
                if (auto_aim_mode != auto_aim_AtkBuff_small)
                    auto_aim_mode = auto_aim_AtkBuff_small;
                else
                    auto_aim_mode = auto_aim_off;
            }
            // b:大符
            if (obj->remote->data.key_single_press_cnt.g != obj->remote->last_data.key_single_press_cnt.g) {
                if (auto_aim_mode != auto_aim_AtkBuff_big)
                    auto_aim_mode = auto_aim_AtkBuff_big;
                else
                    auto_aim_mode = auto_aim_off;
            }

            switch (auto_aim_mode) {
                case auto_aim_off:
                    pc_send_data.auto_mode_flag = 0;
                    break;
                case auto_aim_on:
                    pc_send_data.auto_mode_flag = 1;
                    break;
                case auto_aim_AtkBuff_small:
                    pc_send_data.auto_mode_flag = 2;
                    break;
                case auto_aim_AtkBuff_big:
                    pc_send_data.auto_mode_flag = 3;
                    break;
            }

            if ((auto_aim_mode != auto_aim_off) && (chassis_gimbal_follow_mode == gimbal_follow_chassis)) {
                chassis_gimbal_follow_mode = chassis_follow_gimbal;
            }

            // robot_def
            // 平移
            if (obj->remote->data.key_down.w) obj->send_data.chassis_target.vy = 8000;
            if (obj->remote->data.key_down.s) obj->send_data.chassis_target.vy = -8000;
            if (obj->remote->data.key_down.d) obj->send_data.chassis_target.vx = 8000;
            if (obj->remote->data.key_down.a) obj->send_data.chassis_target.vx = -8000;
            // 按住ctrl减速
            if (obj->remote->data.key_down.ctrl) {
                obj->send_data.chassis_target.vx /= 3;
                obj->send_data.chassis_target.vy /= 3;
            }
            // shift加速
            if (obj->remote->data.key_down.shift) {
                obj->send_data.chassis_target.vx *= 3;
                obj->send_data.chassis_target.vy *= 3;
            }
            // rotate/gimbal
            switch (chassis_gimbal_follow_mode) {
                case gimbal_follow_chassis:
                    obj->send_data.chassis_target.rotate -= 0.5f * (0.7f * (obj->remote->data.mouse.x) + 0.3f * (obj->remote->last_data.mouse.x));
                    if (obj->remote->data.key_down.q) obj->send_data.chassis_target.rotate = -90;
                    if (obj->remote->data.key_down.e) obj->send_data.chassis_target.rotate = 90;
                    obj->send_data.chassis_mode = chassis_run;
                    obj->gimbal_param.mode = gimbal_middle;
                    break;
                case chassis_follow_gimbal:
                    obj->gimbal_param.yaw -= 0.5f * (0.7f * (obj->remote->data.mouse.x) + 0.3f * (obj->remote->last_data.mouse.x));
                    if (obj->remote->data.key_down.q) obj->gimbal_param.yaw -= 15;
                    if (obj->remote->data.key_down.e) obj->gimbal_param.yaw += 15;
                    obj->send_data.chassis_mode = chassis_run_follow_offset;
                    break;
                case independent:
                    obj->gimbal_param.yaw -= 0.5f * (0.7f * (obj->remote->data.mouse.x) + 0.3f * (obj->remote->last_data.mouse.x));
                    if (obj->remote->data.key_down.q) obj->send_data.chassis_target.rotate = -90;
                    if (obj->remote->data.key_down.e) obj->send_data.chassis_target.rotate = 90;
                    break;
            }
            // gimbal_auto_aim

            // shoot
            // 按C开关弹仓
            if (obj->remote->data.key_single_press_cnt.c % 2)
                obj->shoot_param.magazine_mode = magazine_close;
            else
                obj->shoot_param.magazine_mode = magazine_open;
        }

        /*  //电容为可开关模式则解注释
         if (robot->remote->data.key_single_press_cnt.c != robot->remote->last_data.key_single_press_cnt.c)
             robot->send_data.if_supercap_on = 1 - robot->send_data.if_supercap_on; */
        obj->send_data.if_supercap_on = 1;
    }
    // 发布变更
    publish_data gimbal_cmd;
    gimbal_cmd.data = (uint8_t*)&obj->gimbal_param;
    gimbal_cmd.len = sizeof(Cmd_gimbal);
    obj->gimbal_cmd_puber->publish(obj->gimbal_cmd_puber, gimbal_cmd);
    publish_data shoot_cmd;
    shoot_cmd.data = (uint8_t*)&obj->shoot_param;
    shoot_cmd.len = sizeof(Cmd_shoot);
    obj->shoot_cmd_puber->publish(obj->shoot_cmd_puber, shoot_cmd);

    // 小电脑通信
    CanPC_Send(obj->pc, &pc_send_data);

    // 板间通信-发
    if (obj->mode == robot_stop)
        obj->send_data.now_robot_mode = robot_stop;
    else
        obj->send_data.now_robot_mode = robot_run;
    CanSend_Send(obj->send, (uint8_t*)&(obj->send_data));
}

void stop_mode_update(gimbal_board_cmd* obj) {
    obj->send_data.now_robot_mode = robot_stop;
    obj->send_data.chassis_mode = chassis_stop;
    obj->gimbal_param.mode = gimbal_stop;
    obj->shoot_param.mode = shoot_stop;
}

void remote_mode_update(gimbal_board_cmd* obj) {
    // gimbal
    obj->gimbal_param.mode = gimbal_run;
    obj->gimbal_param.yaw -= 0.04f * ((float)obj->remote->data.rc.ch2 - CHx_BIAS);
    obj->gimbal_param.pitch = -1.0f * ((float)obj->remote->data.rc.ch3 - CHx_BIAS);
    obj->gimbal_param.rotate_feedforward = 0;

    // chassis
    // 拨杆确定底盘模式与控制量
    obj->send_data.chassis_target.vy = 16.0f * (float)(obj->remote->data.rc.ch1 - CHx_BIAS);
    obj->send_data.chassis_target.vx = 16.0f * (float)(obj->remote->data.rc.ch0 - CHx_BIAS);
    if (obj->remote->data.rc.s1 == 1) {
        // 小陀螺模式
        obj->send_data.chassis_mode = chassis_rotate_run;
        obj->send_data.chassis_target.vy *= 0.60f;
        obj->send_data.chassis_target.vx *= 0.60f;
    } else {
        // 底盘跟随模式
        obj->send_data.chassis_mode = chassis_run_follow_offset;
    }

    // shoot
    obj->shoot_param.mode = shoot_run;
    if (obj->remote->data.rc.s1 == 2) {
        obj->shoot_param.mode = shoot_stop;
        if (obj->remote->data.rc.ch4 > CHx_BIAS + 400) obj->shoot_param.magazine_mode = magazine_open;
        if (obj->remote->data.rc.ch4 < CHx_BIAS - 400) obj->shoot_param.magazine_mode = magazine_close;
    } else {
        obj->shoot_param.mode = shoot_run;
        obj->shoot_param.bullet_mode = bullet_continuous;
        obj->shoot_param.fire_rate = 0.01f * (float)(obj->remote->data.rc.ch4 - CHx_BIAS);
        obj->shoot_param.heat_limit_remain = obj->recv_data->shoot_referee_data.heat_limit_remain;
        obj->shoot_param.bullet_speed = obj->recv_data->shoot_referee_data.bullet_speed_max;
    }
}

float get_offset_angle(short init_forward, short now_encoder) {
    short tmp = 0;
    if (init_forward < 4096) {
        if (now_encoder > init_forward && now_encoder <= 4096 + init_forward) {
            tmp = now_encoder - init_forward;
        } else if (now_encoder > 4096 + init_forward) {
            tmp = -8192 + now_encoder - init_forward;
        } else {
            tmp = now_encoder - init_forward;
        }
    } else {
        if (now_encoder > init_forward) {
            tmp = now_encoder - init_forward;
        } else if (now_encoder <= init_forward && now_encoder >= init_forward - 4096) {
            tmp = now_encoder - init_forward;
        } else {
            tmp = now_encoder + 8192 - init_forward;
        }
    }
    return tmp * 360.0 / 8192.0;
}