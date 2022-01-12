#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "pub_sub.h"
#include "robot_param.h"
#include "stdint.h"
#include "bsp_log.h"
#include "can_motor.h"
#include "BMI088.h"

typedef struct Gimbal_t {
    BMI088_imu* imu;
    can_motor *pitch, yaw;

    Subscriber* gimbal_cmd_sub;
    Publisher* gimbal_yaw_data_pub;
} Gimbal;

Gimbal* Gimbal_Create(void);
void Gimbal_Update(Gimbal* obj);

#endif