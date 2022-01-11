#ifndef _GIMBAL_H
#define _GIMBAL_H
#include "app.h"

typedef struct Gimbal_param_t {
    float yaw;
    float pitch;
} Gimbal_param;

typedef struct Gimbal_t {
    BMI088_imu* imu;
    can_motor *pitch, yaw;
} Gimbal;

Gimbal* Gimbal_Create(void);
void Gimbal_Update(Gimbal* obj, Gimbal_param config);

#endif