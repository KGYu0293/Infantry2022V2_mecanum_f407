#include "shoot.h"

#include "bsp_pwm.h"

/* 摩擦轮半径(mm) */
#define RADIUS 30
/* 摩擦轮周长(mm) */
#define PERIMETER 94.25f  // 30 * 2 * pi
/* 拨弹电机减速比 */
#define MOTOR_DECELE_RATIO 36.0f
/* 拨弹轮每转一圈发弹数(个) */
#define NUM_PER_CIRCLE 8
/* 每发射一颗小弹增加的热量 */
#define UNIT_HEAT_17MM 10
/* 每发射一颗小弹增加的热量 */
#define UNIT_HEAT_42MM 100

void shoot_motor_lost(void *motor) {
    can_motor *now = (can_motor *)motor;
    printf_log("shoot motor can:%d id:%d lost!\n", now->config.bsp_can_index, now->config.motor_set_id);
}

Shoot *Shoot_Create(void) {
    Shoot *obj = (Shoot *)malloc(sizeof(Shoot));
    obj->load_delta_pos = 8192 * MOTOR_DECELE_RATIO / NUM_PER_CIRCLE;

    //
    obj->shoot_cmd_suber = register_sub("shoot_cmd_topic", 1);

    // 电机初始化
    can_motor_config friction_a_config;
    friction_a_config.motor_model = MODEL_3508;
    friction_a_config.bsp_can_index = 0;
    friction_a_config.motor_set_id = 1;
    friction_a_config.motor_pid_model = SPEED_LOOP;
    friction_a_config.position_fdb_model = MOTOR_FDB;
    friction_a_config.speed_fdb_model = MOTOR_FDB;
    friction_a_config.lost_callback = shoot_motor_lost;
    PID_SetConfig(&friction_a_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&friction_a_config.config_speed, 4, 0.015, 0.8, 2000, 5000);
    obj->friction_a = Can_Motor_Create(&friction_a_config);

    can_motor_config friction_b_config;
    friction_b_config.motor_model = MODEL_3508;
    friction_b_config.bsp_can_index = 0;
    friction_b_config.motor_set_id = 2;
    friction_b_config.motor_pid_model = SPEED_LOOP;
    friction_b_config.position_fdb_model = MOTOR_FDB;
    friction_b_config.speed_fdb_model = MOTOR_FDB;
    friction_b_config.lost_callback = shoot_motor_lost;
    PID_SetConfig(&friction_b_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&friction_b_config.config_speed, 4, 0.015, 0.8, 2000, 5000);
    obj->friction_b = Can_Motor_Create(&friction_b_config);

    can_motor_config load_config;
    load_config.motor_model = MODEL_2006;
    load_config.bsp_can_index = 0;
    load_config.motor_set_id = 3;
    load_config.motor_pid_model = POSITION_LOOP;
    load_config.position_fdb_model = MOTOR_FDB;
    load_config.speed_fdb_model = MOTOR_FDB;
    load_config.lost_callback = shoot_motor_lost;
    PID_SetConfig(&load_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&load_config.config_speed, 20, 0, 0, 2000, 600);
    obj->load = Can_Motor_Create(&load_config);
    return obj;
};

void Shoot_load_motor_set(Shoot *obj, Cmd_shoot *param) {
    if (param->heat_limit_remain < UNIT_HEAT_17MM) {
        param->bullet_mode = bullet_stop;
    }
    switch (param->bullet_mode) {
        case bullet_stop:
            obj->load->config.motor_pid_model = SPEED_LOOP;
            obj->load->speed_pid.ref = 0;
            break;
        case bullet_reverse:  // 反转 防卡弹
            obj->load->config.motor_pid_model = SPEED_LOOP;
            obj->load->speed_pid.ref = 10 * 360 * MOTOR_DECELE_RATIO / NUM_PER_CIRCLE;
            break;
        case bullet_continuous:
            obj->load->config.motor_pid_model = SPEED_LOOP;
            obj->load->speed_pid.ref = -param->fire_rate * 360 * MOTOR_DECELE_RATIO / NUM_PER_CIRCLE;
            break;
        case bullet_single:
            obj->load->config.motor_pid_model = POSITION_LOOP;
            obj->load->position_pid.ref = obj->load->real_position - obj->load_delta_pos;
        case bullet_double:
            obj->load->config.motor_pid_model = POSITION_LOOP;
            obj->load->position_pid.ref = obj->load->real_position - (2 * obj->load_delta_pos);
            break;
        case bullet_trible:
            obj->load->config.motor_pid_model = POSITION_LOOP;
            obj->load->position_pid.ref = obj->load->real_position - (3 * obj->load_delta_pos);
            break;
    }
}

void Shoot_Update(Shoot *obj) {
    // sub并得到param
    publish_data data = obj->shoot_cmd_suber->getdata(obj->shoot_cmd_suber);
    if (data.len == -1) return;  // cmd未发布指令
    Cmd_shoot *param = (Cmd_shoot *)data.data;

    switch (param->mode) {
        case shoot_stop:
            obj->load->enable = MOTOR_STOP;
            obj->friction_a->enable = MOTOR_STOP;
            obj->friction_b->enable = MOTOR_STOP;
            break;
        case shoot_run:
            obj->load->enable = MOTOR_ENABLE;
            obj->friction_a->enable = MOTOR_ENABLE;
            obj->friction_b->enable = MOTOR_ENABLE;

            obj->friction_a->speed_pid.ref = param->bullet_speed * 100 * 360 / RADIUS;
            obj->friction_b->speed_pid.ref = -param->bullet_speed * 100 * 360 / RADIUS;
            // 弹速27.0-27.4的实测ref
            obj->friction_a->speed_pid.ref = 41300;
            obj->friction_b->speed_pid.ref = -41300;
            Shoot_load_motor_set(obj, param);
            break;
    }
    switch (param->magazine_mode) {
        case magazine_open:
            // BSP_PWM_SetCCR();
            break;
        case magazine_close:
            // BSP_PWM_SetCCR();
            break;
    }
}
