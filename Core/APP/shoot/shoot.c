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

cvector *shoot_instances;

can_motor_config friction_a_config;
can_motor_config friction_b_config;
can_motor_config load_config;

void shoot_motor_lost(void *motor) {
    printf_log("shoot motor lost!\n");
    can_motor *now = (can_motor *)motor;
    now->monitor->reset(now->monitor);
}

Shoot *Shoot_Create(void) {
    Shoot *obj = (Shoot *)malloc(sizeof(Shoot));
    obj->load_delta_pos = 8192 * MOTOR_DECELE_RATIO / NUM_PER_CIRCLE;

    // 电机初始化
    friction_a_config.motor_model = MODEL_3508;
    friction_a_config.bsp_can_index = 0;
    friction_a_config.motor_set_id = 1;
    friction_a_config.motor_pid_model = SPEED_LOOP;
    friction_a_config.position_fdb_model = MOTOR_FDB;
    friction_a_config.speed_fdb_model = MOTOR_FDB;
    friction_a_config.lost_callback = shoot_motor_lost;
    PID_SetConfig(&friction_a_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&friction_a_config.config_speed, 20, 0, 0, 2000, 16384);
    obj->friction_a = Can_Motor_Create(&friction_a_config);
    friction_b_config.motor_model = MODEL_3508;
    friction_b_config.bsp_can_index = 0;
    friction_b_config.motor_set_id = 1;
    friction_b_config.motor_pid_model = SPEED_LOOP;
    friction_b_config.position_fdb_model = MOTOR_FDB;
    friction_b_config.speed_fdb_model = MOTOR_FDB;
    friction_b_config.lost_callback = shoot_motor_lost;
    PID_SetConfig(&friction_b_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&friction_b_config.config_speed, 20, 0, 0, 2000, 16384);
    obj->friction_b = Can_Motor_Create(&friction_b_config);
    load_config.motor_model = MODEL_2006;
    load_config.bsp_can_index = 0;
    load_config.motor_set_id = 1;
    load_config.motor_pid_model = POSITION_LOOP;
    load_config.position_fdb_model = MOTOR_FDB;
    load_config.speed_fdb_model = MOTOR_FDB;
    load_config.lost_callback = shoot_motor_lost;
    PID_SetConfig(&load_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&load_config.config_speed, 20, 0, 0, 2000, 16384);
    obj->load = Can_Motor_Create(&load_config);
    return obj;
};

void Shoot_load_motor_set(Shoot *obj, Shoot_param param) {
    if (param.limit.cooling_heat < param.limit.cooling_limit) {
        param.shoot_command = not_fire;
    }
    switch (param.shoot_command) {
        case not_fire:
            load_config.motor_pid_model = POSITION_LOOP;
            obj->load->position_pid.ref = 0;
            break;
        case continuous:
            load_config.motor_pid_model = SPEED_LOOP;
            obj->load->speed_pid.ref = param.fire_rate * 360 * MOTOR_DECELE_RATIO / NUM_PER_CIRCLE;
            break;
        case single:
            load_config.motor_pid_model = POSITION_LOOP;
            obj->load->position_pid.ref = obj->load->real_position + obj->load_delta_pos;
        case Double:
            load_config.motor_pid_model = POSITION_LOOP;
            obj->load->position_pid.ref = obj->load->real_position + (2 * obj->load_delta_pos);
            break;
        case trible:
            load_config.motor_pid_model = POSITION_LOOP;
            obj->load->position_pid.ref = obj->load->real_position + (3 * obj->load_delta_pos);
            break;
        default:
            break;
    }
}

void Shoot_Update(Shoot *obj, Shoot_param param) {
    switch (param.mode) {
        case stop:
            break;
        case run:
            obj->friction_a->speed_pid.ref = param.bullet_speed * 100 * 360 / RADIUS;
            obj->friction_b->speed_pid.ref = -param.bullet_speed * 100 * 360 / RADIUS;
            Shoot_load_motor_set(obj, param);
            break;
        default:
            break;
    }
    switch (param.magazine_lid) {
        case on:
            // BSP_PWM_SetCCR();
            break;
        case off:
            // BSP_PWM_SetCCR();
            break;
        default:
            break;
    }
}
