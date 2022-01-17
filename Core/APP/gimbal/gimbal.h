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
    can_motor *pitch;
    can_motor *yaw;

    Subscriber* gimbal_cmd_sub;
    Publisher* gimbal_upload_pub;
    Gimbal_uplode_data gimbal_upload_data;

    // 陀螺仪->pid参数
    float* yaw_pos_ref;
    float* yaw_spe_ref;
    float* pitch_pos_ref;
    float* pitch_spe_ref;
} Gimbal;

Gimbal* Gimbal_Create(void);
void Gimbal_Update(Gimbal* obj);

#endif