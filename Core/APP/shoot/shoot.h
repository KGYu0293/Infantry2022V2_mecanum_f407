#ifndef _SHOOT_H_
#define _SHOOT_H_
#include "app.h"

typedef struct Shoot_restrict_data_t{
    uint16_t cooling_heat;
    float cooling_limit;
} Shoot_limit;

typedef struct Shoot_param_t {
    enum { stop, run } mode;
    enum { not_fire,single, Double, trible, continuous } shoot_command;
    enum { on, off } magazine_lid;  // 弹仓盖
    uint16_t bullet_speed;// 弹速
    float fire_rate;// 射频（发/秒）
    Shoot_limit limit;
} Shoot_param;


typedef struct Shoot_t {
    can_motor *friction_a;
    can_motor *friction_b;
    can_motor *load;

    int load_delta_pos;
} Shoot;

Shoot *Shoot_Create(void);
void Shoot_Update(Shoot *obj, Shoot_param param);
#endif