#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
#include "bsp_log.h"
#include "can_motor.h"
#include "BMI088.h"

typedef struct Gimbal_t {
    BMI088_imu* imu;
    can_motor *pitch;
    can_motor *yaw;

    Subscriber* gimbal_cmd_sub;
    Publisher* gimbal_upload_pub;
    Upload_gimbal gimbal_upload_data;
    Cmd_gimbal* cmd_data;

    float pitch_limit_down;
    float pitch_limit_up;
} Gimbal;

Gimbal* Gimbal_Create(void);
void Gimbal_Update(Gimbal* obj);

#endif