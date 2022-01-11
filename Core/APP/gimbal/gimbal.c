#include "gimbal.h"

cvector *gimbal_instances;

can_motor_config yaw_config;
can_motor_config pitch_config;

Gimbal *Gimbal_Create() {
    Gimbal *obj = (Gimbal *)malloc(sizeof(Gimbal));
    // 电机初始化
    yaw_config.motor_model = MODEL_6020;
    yaw_config.bsp_can_index = 0;
    yaw_config.motor_set_id = 1;
    yaw_config.motor_pid_model = SPEED_LOOP;
    yaw_config.position_fdb_model = MOTOR_FDB;
    yaw_config.speed_fdb_model = MOTOR_FDB;
    yaw_config.lost_callback = NULL;
    PID_SetConfig(&yaw_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&yaw_config.config_speed, 20, 0, 0, 2000, 12000);
    pitch_config.motor_model = MODEL_6020;
    pitch_config.bsp_can_index = 0;
    pitch_config.motor_set_id = 1;
    pitch_config.motor_pid_model = SPEED_LOOP;
    pitch_config.position_fdb_model = MOTOR_FDB;
    pitch_config.speed_fdb_model = MOTOR_FDB;
    pitch_config.lost_callback = NULL;
    PID_SetConfig(&pitch_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&pitch_config.config_speed, 20, 0, 0, 2000, 12000);

    return obj;
}