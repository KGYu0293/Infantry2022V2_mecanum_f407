#ifndef _SHOOT_H_
#define _SHOOT_H_

#include "bsp_log.h"
#include "can_motor.h"
#include "pub_sub.h"
#include "pwm_servo.h"
#include "robot_def.h"
#include "stdint.h"

#pragma pack(1)
typedef struct Shoot_t {
    can_motor *friction_a;
    can_motor *friction_b;
    can_motor *load;

    Servo *mag_lid;

    Subscriber *shoot_cmd_suber;
    Cmd_shoot *cmd_data;
    Publisher *shoot_upload_puber;
    Upload_shoot upload_data;

    //用于单/双/三发模式
    uint32_t cooldown_start;  //冷却起始时间点
    uint32_t cooldown_time;   //冷却时间

    // 补偿控制
    circular_queue *bullet_speed_queue;
    uint32_t shoot_time;
    uint16_t bullet_cnt;
    uint16_t last_bullet_cnt;
    float shoot_speed_reduce;
} Shoot;
#pragma pack()

Shoot *Shoot_Create(void);
void Shoot_Update(Shoot *obj);
#endif