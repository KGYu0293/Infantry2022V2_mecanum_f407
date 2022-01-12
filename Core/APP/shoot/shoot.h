#ifndef _SHOOT_H_
#define _SHOOT_H_

#include "pub_sub.h"
#include "robot_param.h"
#include "stdint.h"
#include "bsp_log.h"
#include "can_motor.h"

typedef struct Shoot_t {
    can_motor *friction_a;
    can_motor *friction_b;
    can_motor *load;

    int load_delta_pos;
} Shoot;

Shoot *Shoot_Create(void);
void Shoot_Update(Shoot *obj);
#endif