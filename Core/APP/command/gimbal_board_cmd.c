#include "gimbal_board_cmd.h"

#include "bsp.h"
#define INIT_FORWARD 3152  // 云台朝向底盘正前时云台yaw编码器值
// monitor处理函数
void gimbal_core_module_lost(void* obj) { printf_log("gimbal_core_module_lost!!!robot stopped for security.\n"); }
void pc_lost(void* obj) { printf_log("pc lost!\n"); }

// cmd的private函数
void stop_mode_update(gimbal_board_cmd* obj);       //机器人停止模式更新函数
void remote_mode_update(gimbal_board_cmd* obj);     //机器人遥控器模式更新函数
void mouse_key_mode_update(gimbal_board_cmd* obj);  //机器人键鼠模式更新函数
void send_cmd_and_data(gimbal_board_cmd* obj);      //发布指令和板间通信
void mousekey_GimbalChassis_default(gimbal_board_cmd* obj);
// 其他功能函数
float get_offset_angle(short init_forward, short now_encoder);  // 获取云台朝向与底盘正前的夹角

gimbal_board_cmd* Gimbal_board_CMD_Create() {
    // 创建实例
    gimbal_board_cmd* obj = (gimbal_board_cmd*)malloc(sizeof(gimbal_board_cmd));
    memset(obj, 0, sizeof(gimbal_board_cmd));

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
    obj->recv_data = (Chassis_board_send_data*)obj->recv->data_rx.data;

    // 小电脑通信配置
    canpc_config pc_config;
    pc_config.bsp_can_index = 0;
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
    memset(&(obj->pc_send_data), 0, sizeof(canpc_send));
    memset(&(obj->gimbal_control), 0, sizeof(Cmd_gimbal));
    memset(&(obj->shoot_control), 0, sizeof(Cmd_shoot));
    memset(&(obj->send_data), 0, sizeof(Gimbal_board_send_data));
    obj->send_data.if_consume_supercap = 0;  //默认关闭电容输出
    obj->robot_ready = 0;
    obj->chassis_climb_mode = 0;
    obj->autoaim_mode = auto_aim_off;
    return obj;
}

void Gimbal_board_CMD_Update(gimbal_board_cmd* obj) {
    // 判断机器人工作模式
    //初始化为RUN
    obj->mode = robot_run;

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
        obj->gimbal_upload_data = (Upload_gimbal*)gimbal_data_fdb.data;
        obj->pc_send_data.euler[0] = obj->gimbal_upload_data->gimbal_imu->euler[0];
        obj->pc_send_data.euler[1] = obj->gimbal_upload_data->gimbal_imu->euler[1];
        obj->pc_send_data.euler[2] = obj->gimbal_upload_data->gimbal_imu->euler[2];
        if (obj->gimbal_upload_data->gimbal_status == module_lost) obj->mode = robot_stop;
    }

    // 除了遥控器之外都已经上线
    if (obj->mode == robot_run) {
        if (!obj->robot_ready) {
            Buzzer_Start(obj->internal_buzzer);
        }
        obj->robot_ready = 1;
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
        obj->send_data.chassis_target.offset_angle = get_offset_angle(INIT_FORWARD, *obj->gimbal_upload_data->yaw_encorder);
        // 自瞄关
        obj->pc_send_data.auto_mode_flag = 0;
        // 遥控器控制模式
        if (obj->remote->data.input_mode == RC_Remote) {
            remote_mode_update(obj);
        }
        // 键鼠控制模式
        else if (obj->remote->data.input_mode == RC_MouseKey) {
            mouse_key_mode_update(obj);
        }

        // 板间通信-发
        obj->send_data.now_robot_mode = robot_run;
    }

    /*  //电容为可开关模式则解注释
     if (robot->remote->data.key_single_press_cnt.c != robot->remote->last_data.key_single_press_cnt.c)
         robot->send_data.if_supercap_on = 1 - robot->send_data.if_supercap_on; */
    // obj->send_data.if_supercap_on = 1;

    // 发布控制结果和通信
    send_cmd_and_data(obj);
}

void mousekey_GimbalChassis_default(gimbal_board_cmd* obj) {
    obj->gimbal_control.mode = gimbal_run;
    obj->send_data.chassis_mode = chassis_run_follow_offset;
}

void stop_mode_update(gimbal_board_cmd* obj) {
    obj->send_data.now_robot_mode = robot_stop;
    obj->send_data.chassis_mode = chassis_stop;
    obj->gimbal_control.mode = gimbal_stop;
    obj->shoot_control.mode = shoot_stop;
}

void remote_mode_update(gimbal_board_cmd* obj) {
    // 云台控制参数
    obj->gimbal_control.mode = gimbal_run;
    obj->gimbal_control.yaw -= 0.04f * ((float)obj->remote->data.rc.ch2 - CHx_BIAS);
    obj->gimbal_control.pitch = -1.0f * ((float)obj->remote->data.rc.ch3 - CHx_BIAS);
    obj->gimbal_control.rotate_feedforward = 0;  // 目前没有使用小陀螺前馈

    // 底盘控制参数
    // 拨杆确定底盘模式与控制量
    obj->send_data.if_consume_supercap = 0;  //遥控器模式不消耗超级电容
    obj->send_data.chassis_target.vy = 16.0f * (float)(obj->remote->data.rc.ch1 - CHx_BIAS);
    obj->send_data.chassis_target.vx = 16.0f * (float)(obj->remote->data.rc.ch0 - CHx_BIAS);
    if (obj->remote->data.rc.s1 == 2) {
        // 小陀螺模式
        obj->send_data.chassis_mode = chassis_rotate_run;
        obj->send_data.chassis_target.vy *= 0.60f;
        obj->send_data.chassis_target.vx *= 0.60f;
    } else {
        // 底盘跟随模式
        obj->send_data.chassis_mode = chassis_run_follow_offset;
    }

    // 发射机构控制
    obj->shoot_control.mode = shoot_run;
    if (obj->remote->data.rc.s1 == 1) {
        obj->shoot_control.bullet_mode = bullet_holdon;
        obj->shoot_control.bullet_speed = 0;
        if (obj->remote->data.rc.ch4 > CHx_BIAS + 400) obj->shoot_control.mag_mode = magazine_open;
        if (obj->remote->data.rc.ch4 < CHx_BIAS - 400) obj->shoot_control.mag_mode = magazine_close;
    } else {
        obj->shoot_control.bullet_mode = bullet_continuous;
        obj->shoot_control.fire_rate = 0.01f * (float)(obj->remote->data.rc.ch4 - CHx_BIAS);
        obj->shoot_control.heat_limit_remain = obj->recv_data->shoot_referee_data.heat_limit_remain;
        obj->shoot_control.bullet_speed = obj->recv_data->shoot_referee_data.bullet_speed_max;
    }
}

void mouse_key_mode_update(gimbal_board_cmd* obj) {
    // static enum { chassis_follow_gimbal, gimbal_follow_chassis, independent } chassis_gimbal_follow_mode = chassis_follow_gimbal;

    // 云台-底盘运动模式
    // 按一下r:小陀螺
    if (obj->remote->data.key_single_press_cnt.r != obj->remote->last_data.key_single_press_cnt.r) {
        if (obj->send_data.chassis_mode != chassis_rotate_run) {
            obj->send_data.chassis_mode = chassis_rotate_run;  // 小陀螺模式
            obj->gimbal_control.mode = gimbal_run;             // 云台正常模式
        } else {
            mousekey_GimbalChassis_default(obj);
        }
    }
    // v:云台底盘独立
    if (obj->remote->data.key_single_press_cnt.v != obj->remote->last_data.key_single_press_cnt.v) {
        if (obj->send_data.chassis_mode != chassis_run || obj->gimbal_control.mode != gimbal_run) {
            obj->send_data.chassis_mode = chassis_run;
            obj->gimbal_control.mode = gimbal_run;
        } else {
            mousekey_GimbalChassis_default(obj);
        }
    }
    // x:云台跟随底盘
    if (obj->remote->data.key_single_press_cnt.x != obj->remote->last_data.key_single_press_cnt.x) {
        if (obj->gimbal_control.mode != gimbal_middle || obj->send_data.chassis_mode != chassis_run) {
            obj->gimbal_control.mode = gimbal_middle;
            obj->send_data.chassis_mode = chassis_run;
        } else {
            mousekey_GimbalChassis_default(obj);
        }
    }
    // z:底盘超限模式
    if (obj->remote->data.key_single_press_cnt.z != obj->remote->last_data.key_single_press_cnt.z) {
        obj->chassis_climb_mode ^= 1;  //反转状态
    }

    // 自瞄模式
    // f:自瞄
    if (obj->remote->data.key_single_press_cnt.f != obj->remote->last_data.key_single_press_cnt.f) {
        if (obj->autoaim_mode != auto_aim_normal)
            obj->autoaim_mode = auto_aim_normal;
        else
            obj->autoaim_mode = auto_aim_off;
    }
    // g:小符
    if (obj->remote->data.key_single_press_cnt.g != obj->remote->last_data.key_single_press_cnt.g) {
        if (obj->autoaim_mode != auto_aim_buff_small)
            obj->autoaim_mode = auto_aim_buff_small;
        else
            obj->autoaim_mode = auto_aim_off;
    }
    // b:大符
    if (obj->remote->data.key_single_press_cnt.g != obj->remote->last_data.key_single_press_cnt.g) {
        if (obj->autoaim_mode != auto_aim_buff_big)
            obj->autoaim_mode = auto_aim_buff_big;
        else
            obj->autoaim_mode = auto_aim_off;
    }
    obj->pc_send_data.auto_mode_flag = obj->autoaim_mode;

    // 底盘控制参数
    obj->send_data.if_consume_supercap = 0;  //默认不要消耗电容电量
    obj->send_data.chassis_target.vx = 0;
    obj->send_data.chassis_target.vy = 0;
    obj->send_data.chassis_target.rotate = 0;
    // 平移
    if (obj->remote->data.key_down.w) obj->send_data.chassis_target.vy = 1500;
    if (obj->remote->data.key_down.s) obj->send_data.chassis_target.vy = -1500;
    if (obj->remote->data.key_down.d) obj->send_data.chassis_target.vx = -1500;
    if (obj->remote->data.key_down.a) obj->send_data.chassis_target.vx = 1500;
    // 按住ctrl减速
    if (obj->remote->data.key_down.ctrl) {
        obj->send_data.chassis_target.vx /= 2.0;
        obj->send_data.chassis_target.vy /= 2.0;
    }
    // 按住shift加速/飞坡模式加速
    if (obj->remote->data.key_down.shift || obj->chassis_climb_mode) {
        obj->send_data.chassis_target.vx *= 2.0;
        obj->send_data.chassis_target.vy *= 2.0;
        if (obj->chassis_climb_mode) {
            obj->send_data.chassis_target.vx *= 2.0;
            obj->send_data.chassis_target.vy *= 2.0;
        }
        obj->send_data.if_consume_supercap = 1;
    }

    // q/e:底盘转向
    if (obj->remote->data.key_down.q) {
        if (obj->send_data.chassis_mode == chassis_run) {
            obj->send_data.chassis_target.rotate = +60;
        } else if (obj->send_data.chassis_mode == chassis_run_follow_offset) {
            obj->gimbal_control.yaw += 7;
        }
    }
    if (obj->remote->data.key_down.e) {
        if (obj->send_data.chassis_mode == chassis_run) {
            obj->send_data.chassis_target.rotate = -60;
        } else if (obj->send_data.chassis_mode == chassis_run_follow_offset) {
            obj->gimbal_control.yaw -= 7;
        }
    }

    // 云台控制参数
    if (obj->gimbal_control.mode == gimbal_run) {
        if (obj->autoaim_mode == auto_aim_off) {
            obj->gimbal_control.yaw -= 0.3f * (0.7f * (obj->remote->data.mouse.x) + 0.3f * (obj->remote->last_data.mouse.x));
            obj->gimbal_control.pitch += 0.1f * ((float)obj->remote->data.mouse.y);
            //不管开没开自瞄，都更新pc接收数据的状态
            if (*obj->pc->data_updated) {
                *obj->pc->data_updated = 0;
            }
        } else {
            static int16_t pc_lost_cnt = 10;
            if (*obj->pc->data_updated) {
                *obj->pc->data_updated = 0;
                // 自瞄开
                // 计算真实yaw值
                if(obj->pc->pc_recv_data->wait_time >= 0){
                    float yaw_target = obj->pc->pc_recv_data->yaw * 8192.0 / 2 / pi + obj->gimbal_upload_data->gimbal_imu->round * 8192.0;
                    if (obj->pc->pc_recv_data->yaw - obj->gimbal_upload_data->gimbal_imu->euler[2] > pi) yaw_target -= 8192;
                    if (obj->pc->pc_recv_data->yaw - obj->gimbal_upload_data->gimbal_imu->euler[2] < -pi) yaw_target += 8192;
                    obj->gimbal_control.yaw = yaw_target;
                    obj->gimbal_control.pitch = obj->pc->pc_recv_data->roll * 8192.0 / 2 / pi;  // 根据当前情况决定，pitch轴反馈为陀螺仪roll
                } else {
                    // 没有目标
                    //使用鼠标控制云台
                    obj->gimbal_control.yaw -= 0.3f * (0.7f * (obj->remote->data.mouse.x) + 0.3f * (obj->remote->last_data.mouse.x));
                    obj->gimbal_control.pitch += 0.1f * ((float)obj->remote->data.mouse.y);
                }
                
                pc_lost_cnt = 10;
            } else {
                pc_lost_cnt--;
                //判断小电脑通信彻底丢失（等待了20ms）
                if (pc_lost_cnt <= 0) {
                    pc_lost_cnt = 0;
                    //使用鼠标控制云台
                    obj->gimbal_control.yaw -= 0.3f * (0.7f * (obj->remote->data.mouse.x) + 0.3f * (obj->remote->last_data.mouse.x));
                    obj->gimbal_control.pitch += 0.1f * ((float)obj->remote->data.mouse.y);
                }
            }
        }
    } else if (obj->gimbal_control.mode == gimbal_middle) {
        // 云台跟随底盘模式
    }

    // c:开关弹仓
    if (obj->remote->data.key_single_press_cnt.c % 2)
        obj->shoot_control.mag_mode = magazine_close;
    else
        obj->shoot_control.mag_mode = magazine_open;
    // 发射机构控制参数
    obj->shoot_control.mode = shoot_run;
    if (obj->remote->data.rc.s1 == 1) {
        // 发射机构刹车
        obj->shoot_control.bullet_mode = bullet_holdon;
        obj->shoot_control.bullet_speed = 0;
    } else {
        // 发弹控制，单发，双发, 射频和小电脑控制待完善
        obj->shoot_control.heat_limit_remain = obj->recv_data->shoot_referee_data.heat_limit_remain;  // 下板传回的热量剩余
        obj->shoot_control.bullet_speed = obj->recv_data->shoot_referee_data.bullet_speed_max;        // 下板传回的子弹速度上限
        obj->shoot_control.fire_rate = 3;                                                             // 固定射频
        if (obj->remote->data.mouse.press_l) {
            obj->shoot_control.bullet_mode = bullet_continuous;
        } else {
            obj->shoot_control.bullet_mode = bullet_holdon;
        }
    }
}

void send_cmd_and_data(gimbal_board_cmd* obj) {
    CanPC_Send(obj->pc, &obj->pc_send_data);               // 小电脑通信
    CanSend_Send(obj->send, (uint8_t*)&(obj->send_data));  // 板间通信
    // 子模块pub_sub
    publish_data gimbal_cmd;
    gimbal_cmd.data = (uint8_t*)&obj->gimbal_control;
    gimbal_cmd.len = sizeof(Cmd_gimbal);
    obj->gimbal_cmd_puber->publish(obj->gimbal_cmd_puber, gimbal_cmd);
    publish_data shoot_cmd;
    shoot_cmd.data = (uint8_t*)&obj->shoot_control;
    shoot_cmd.len = sizeof(Cmd_shoot);
    obj->shoot_cmd_puber->publish(obj->shoot_cmd_puber, shoot_cmd);
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