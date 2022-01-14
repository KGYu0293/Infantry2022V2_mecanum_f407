#include "gimbal.h"

#define GIMBAL_MOTOR_ENCORDER_MIDDLE
#define YAW_MOTOR_ENCORDER_MIDDLE
void gimbal_motor_lost(void *motor) { printf_log("gimbal motor lost!\n"); }

Gimbal *Gimbal_Create() {
    Gimbal *obj = (Gimbal *)malloc(sizeof(Gimbal));
    //外设初始化
    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 55.0f;  //设定温度为55度
    internal_imu_config.lost_callback = NULL;
    obj->imu = BMI088_Create(&internal_imu_config);
    
    can_motor_config yaw_config;
    yaw_config.motor_model = MODEL_6020;
    yaw_config.bsp_can_index = 0;
    yaw_config.motor_set_id = 1;
    yaw_config.motor_pid_model = SPEED_LOOP;
    yaw_config.position_fdb_model = MOTOR_FDB;
    yaw_config.speed_fdb_model = MOTOR_FDB;
    yaw_config.lost_callback = gimbal_motor_lost;
    PID_SetConfig(&yaw_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&yaw_config.config_speed, 20, 0, 0, 2000, 12000);
    obj->yaw = Can_Motor_Create(&yaw_config);
    can_motor_config pitch_config;
    pitch_config.motor_model = MODEL_6020;
    pitch_config.bsp_can_index = 0;
    pitch_config.motor_set_id = 1;
    pitch_config.motor_pid_model = SPEED_LOOP;
    pitch_config.position_fdb_model = MOTOR_FDB;
    pitch_config.speed_fdb_model = MOTOR_FDB;
    pitch_config.lost_callback = gimbal_motor_lost;
    PID_SetConfig(&pitch_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&pitch_config.config_speed, 20, 0, 0, 2000, 12000);
    obj->pitch = Can_Motor_Create(&pitch_config);

    // 定义sub
    obj->gimbal_cmd_sub = register_sub(gimbal_cmd_topic, sizeof(Gimbal_param));
    // 定义pub
    obj->gimbal_yaw_data_pub = register_pub(gimbal_upload_topic);
    return obj;
}

void Gimbal_calculate(Gimbal *obj, Gimbal_param *param) {
    obj->yaw->position_pid.ref = param->yaw;
    obj->pitch->position_pid.ref = param->pitch;
}

void Gimbal_Update(Gimbal *gimbal) {
    // 反馈yaw编码器信息
    publish_data gimbal_uplode;
    gimbal_uplode.data = (uint8_t*)&gimbal->yaw->fdbPosition;
    gimbal_uplode.len = sizeof(gimbal->yaw->fdbPosition);
    gimbal->gimbal_yaw_data_pub->publish(gimbal->gimbal_yaw_data_pub, gimbal_uplode);

    // 取得控制参数
    publish_data gimbal_data = gimbal->gimbal_cmd_sub->getdata(gimbal->gimbal_cmd_sub);
    if (gimbal_data.len == -1) return;
    Gimbal_param *param = (Gimbal_param *)gimbal_data.data;

    // 模块控制
    switch (param->mode) {
        case gimbal_stop:
            gimbal->pitch->enable = MOTOR_STOP;
            gimbal->yaw->enable = MOTOR_STOP;
            break;
        case gimbal_run:
            gimbal->pitch->enable = MOTOR_ENABLE;
            gimbal->yaw->enable = MOTOR_ENABLE;
            Gimbal_calculate(gimbal, param);
            break;
        default:
            break;
    }
}