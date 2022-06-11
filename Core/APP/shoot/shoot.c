#include "shoot.h"

// #include "bsp_pwm.h"
#include "bsp.h"
#include "bsp_time.h"

void shoot_motor_lost(void *motor) {
    can_motor *now = (can_motor *)motor;
    printf_log("shoot motor can:%d id:%d lost!\n", now->config.bsp_can_index, now->config.motor_set_id);
}

Shoot *Shoot_Create(void) {
    Shoot *obj = (Shoot *)malloc(sizeof(Shoot));
    memset(obj, 0, sizeof(Shoot));

    // 电机初始化
    can_motor_config friction_a_config;
    controller_config friction_a_controller_config;
    friction_a_controller_config.control_type = PID_MODEL;
    friction_a_controller_config.control_depth = SPEED_CONTROL;
    PID_SetConfig_Pos(&friction_a_controller_config.position_pid_config, 0, 0, 0, 0, 0);
    PID_SetConfig_Comp(&friction_a_controller_config.speed_pid_config, 2.5, 1.2, 0.015, 0.6, 1600, 400, 1200, 3000, 2000, 15000);
    friction_a_config.motor_model = MODEL_3508;
    friction_a_config.bsp_can_index = 0;
    friction_a_config.motor_set_id = 1;
    friction_a_config.motor_controller_config = friction_a_controller_config;
    friction_a_config.position_fdb_model = MOTOR_FDB;
    friction_a_config.speed_fdb_model = MOTOR_FDB;
    friction_a_config.output_model = MOTOR_OUTPUT_NORMAL;
    friction_a_config.lost_callback = shoot_motor_lost;
    obj->friction_a = Can_Motor_Create(&friction_a_config);


    can_motor_config friction_b_config;
    controller_config friction_b_controller_config;
    friction_b_controller_config.control_type = PID_MODEL;
    friction_b_controller_config.control_depth = SPEED_CONTROL;
    PID_SetConfig_Pos(&friction_b_controller_config.position_pid_config, 0, 0, 0, 0, 0);
    PID_SetConfig_Comp(&friction_b_controller_config.speed_pid_config, 2.5, 1.2, 0.015, 0.6, 1600, 400, 1200, 3000, 2000, 15000);
    friction_b_config.motor_model = MODEL_3508;
    friction_b_config.bsp_can_index = 0;
    friction_b_config.motor_set_id = 2;
    friction_b_config.motor_controller_config = friction_b_controller_config;
    friction_b_config.position_fdb_model = MOTOR_FDB;
    friction_b_config.speed_fdb_model = MOTOR_FDB;
    friction_b_config.output_model = MOTOR_OUTPUT_NORMAL;
    friction_b_config.lost_callback = shoot_motor_lost;
    obj->friction_b = Can_Motor_Create(&friction_b_config);

    can_motor_config load_config;
    controller_config load_controller_config;
    load_controller_config.control_type = PID_MODEL;
    load_controller_config.control_depth = POS_CONTROL;
    PID_SetConfig_Pos(&load_controller_config.position_pid_config, 44, 0, 2, 2000, 50000);
    PID_SetConfig_Pos(&load_controller_config.speed_pid_config, 2, 0, 0.5, 2000, 10000);
    load_config.motor_model = MODEL_2006;
    load_config.bsp_can_index = 0;
    load_config.motor_set_id = 3;
    load_config.motor_controller_config = load_controller_config;
    load_config.position_fdb_model = MOTOR_FDB;
    load_config.speed_fdb_model = MOTOR_FDB;
    load_config.output_model = MOTOR_OUTPUT_NORMAL;
    load_config.lost_callback = shoot_motor_lost;
    
    obj->load = Can_Motor_Create(&load_config);

    // 舵机
    Servo_config magazine_config;
    magazine_config.model = MODEL_POS;
    magazine_config.bsp_pwm_index = PWM_SERVO_1_PORT;
    magazine_config.max_angle = 270;
    magazine_config.initial_angle = 100;
    obj->mag_lid = Servo_Create(&magazine_config);

    // 关闭红点激光
    BSP_GPIO_Set(GPIO_5V_OUTPUT, 0);

    obj->shoot_cmd_suber = register_sub("cmd_shoot", 1);
    obj->shoot_upload_puber = register_pub("upload_shoot");
    obj->cmd_data = NULL;
    obj->cooldown_start = obj->cooldown_time = 0;
    return obj;
};

void Shoot_load_Update(Shoot *obj, Cmd_shoot *param) {
    if (param->heat_limit_remain <= SHOOT_UNIT_HEAT_17MM * 3) {
        param->bullet_mode = bullet_holdon;
    }
    // 发射一个弹丸编码器转过的角度
    static float load_delta_pos = 360.0 * SHOOT_MOTOR_DECELE_RATIO / SHOOT_NUM_PER_CIRCLE;

    uint32_t time_now = BSP_sys_time_ms();
    if (time_now < obj->cooldown_start + obj->cooldown_time) return;
    switch (param->bullet_mode) {
        case bullet_holdon:
            obj->load->motor_controller->config.control_depth = POS_CONTROL;
            obj->load->motor_controller->ref_position = obj->load->real_position;  // 刹车
            break;
        case bullet_reverse:  // 反转 防卡弹
            obj->load->motor_controller->config.control_depth = SPEED_CONTROL;
            obj->load->motor_controller->ref_speed = 5 * 360 * SHOOT_MOTOR_DECELE_RATIO / SHOOT_NUM_PER_CIRCLE;
            break;
        case bullet_continuous:
            // obj->load->config.motor_pid_model = SPEED_LOOP;
            // obj->load->speed_pid.ref = -param->fire_rate * 360 * SHOOT_MOTOR_DECELE_RATIO / SHOOT_NUM_PER_CIRCLE;
            obj->load->motor_controller->config.control_depth = POS_CONTROL;
            if (param->fire_rate < 2) {
                obj->load->motor_controller->ref_position = obj->load->real_position;
            } else {
                obj->load->motor_controller->ref_position = obj->load->real_position - load_delta_pos;
                obj->cooldown_start = time_now;
                obj->cooldown_time = (int)(1000 / param->fire_rate);  //待测试
            }
            break;
        case bullet_single:
            obj->load->motor_controller->config.control_depth = POS_CONTROL;
            obj->load->motor_controller->ref_position = obj->load->real_position - load_delta_pos;
            obj->cooldown_start = time_now;
            obj->cooldown_time = 300;  //待测试
            break;
        case bullet_double:
            obj->load->motor_controller->config.control_depth = POS_CONTROL;
            obj->load->motor_controller->ref_position = obj->load->real_position - (2 * load_delta_pos);
            obj->cooldown_start = time_now;
            obj->cooldown_time = 450;  //待测试
            break;
        case bullet_trible:
            obj->load->motor_controller->config.control_depth = POS_CONTROL;
            obj->load->motor_controller->ref_position = obj->load->real_position - (3 * load_delta_pos);
            obj->cooldown_start = time_now;
            obj->cooldown_time = 650;  //待测试
            break;
    }
}

void Shoot_Update(Shoot *obj) {
    // sub并得到param
    publish_data data = obj->shoot_cmd_suber->getdata(obj->shoot_cmd_suber);
    if (data.len == -1) return;  // cmd未发布指令
    obj->cmd_data = (Cmd_shoot *)data.data;

    switch (obj->cmd_data->mode) {
        case shoot_stop:
            obj->load->enable = MOTOR_STOP;
            obj->friction_a->enable = MOTOR_STOP;
            obj->friction_b->enable = MOTOR_STOP;
            break;
        case shoot_run:
            obj->load->enable = MOTOR_ENABLE;
            obj->friction_a->enable = MOTOR_ENABLE;
            obj->friction_b->enable = MOTOR_ENABLE;
            switch (obj->cmd_data->bullet_speed) {
                case 30:
                    // 弹速28.7-29.2的实测ref 28.6
                    obj->friction_a->motor_controller->ref_speed = 42300;
                    obj->friction_b->motor_controller->ref_speed = -42300;
                    obj->upload_data.real_bullet_speed = 28.8;// 此处填写该case下调得实际弹速的典型值
                    break;
                case 18:
                    // 17.0-17.8
                    obj->friction_a->motor_controller->ref_speed = 29700;
                    obj->friction_b->motor_controller->ref_speed = -29700;
                    obj->upload_data.real_bullet_speed = 17.3;
                    break;
                case 15:
                    // 偏差较大 有不少14.6
                    obj->friction_a->motor_controller->ref_speed = 27000;
                    obj->friction_b->motor_controller->ref_speed = -27000;
                    obj->upload_data.real_bullet_speed = 14.1;
                    break;
                case 0:  // 刹车
                    obj->friction_a->motor_controller->ref_speed = 0;
                    obj->friction_b->motor_controller->ref_speed = 0;
                    obj->upload_data.real_bullet_speed = 0;
                default:
                    //待实测
                    obj->friction_a->motor_controller->ref_speed = 27700;
                    obj->friction_b->motor_controller->ref_speed = -27700;
                    obj->upload_data.real_bullet_speed = 14.1;
                    break;
            }
            Shoot_load_Update(obj, obj->cmd_data);
            break;
    }
    switch (obj->cmd_data->mag_mode) {
        case magazine_open:
            obj->mag_lid->pos_servo_control = 10;
            break;
        case magazine_close:
            obj->mag_lid->pos_servo_control = 114;
            break;
    }

    // 返回实际弹速
    publish_data shoot_upload;
    shoot_upload.data = (uint8_t *)&(obj->upload_data);
    shoot_upload.len  = sizeof(Upload_shoot);
    obj->shoot_upload_puber->publish(obj->shoot_upload_puber, shoot_upload);
}
