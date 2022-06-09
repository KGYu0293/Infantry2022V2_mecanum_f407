#ifndef _SHOOT_H_
#define _SHOOT_H_

#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
#include "bsp_log.h"
#include "can_motor.h"
#include "pwm_servo.h"

typedef struct Shoot_t {
    can_motor *friction_a;
    can_motor *friction_b;
    can_motor *load;

    Servo *mag_lid;

    Subscriber* shoot_cmd_suber;
    Cmd_shoot *cmd_data;
    Publisher* shoot_upload_puber;
    Upload_shoot upload_data;

    //用于单/双/三发模式
    uint32_t cooldown_start;  //冷却起始时间点
    uint32_t cooldown_time;   //冷却时间
} Shoot;

Shoot *Shoot_Create(void);
void Shoot_Update(Shoot *obj);
#endif