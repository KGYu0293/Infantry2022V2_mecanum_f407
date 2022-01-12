#include "gimbal.h"

void gimbal_motor_lost(void *motor) { printf_log("gimbal motor lost!\n"); }

Gimbal *Gimbal_Create() {
    Gimbal *obj = (Gimbal *)malloc(sizeof(Gimbal));
    // 电机初始化
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

    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 55.0f;  //设定温度为55度
    internal_imu_config.lost_callback = NULL;
    obj->imu = BMI088_Create(&internal_imu_config);

    // 定义sub
    obj->gimbal_cmd_sub = register_sub(gimbal_cmd_topic, sizeof(Gimbal_param));
    // 定义pub
    obj->gimbal_yaw_data_pub = register_pub(gimbal_uplode_topic);
    return obj;
}