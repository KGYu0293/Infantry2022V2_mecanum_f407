#include "gimbal.h"

// 云台水平并朝向底盘正前方时云台和底盘的编码器值
#define PITCH_MOTOR_ENCORDER_BIAS 841
#define YAW_MOTOR_ENCORDER_BIAS 3153
// 云台抬头/低头限位
#define PITCH_ENCORDER_HIGHEST (PITCH_MOTOR_ENCORDER_BIAS + 500)
#define PITCH_ENCORDER_LOWEST (PITCH_MOTOR_ENCORDER_BIAS - 500)

void gimbal_motor_lost(void *motor) { printf_log("gimbal motor lost!\n"); }
void gimbal_imu_lost(void *imu) { printf_log("gimbal imu lost!!!gimbal stopped.\n"); }

Gimbal *Gimbal_Create() {
    Gimbal *obj = (Gimbal *)malloc(sizeof(Gimbal));
    //外设初始化
    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 55.0f;  //设定温度为55度
    internal_imu_config.lost_callback = gimbal_imu_lost;
    obj->imu = BMI088_Create(&internal_imu_config);

    can_motor_config yaw_config;
    yaw_config.motor_model = MODEL_6020;
    yaw_config.bsp_can_index = 1;  // 1
    yaw_config.motor_set_id = 1;   // 1
    yaw_config.motor_pid_model = POSITION_LOOP;
    yaw_config.position_fdb_model = OTHER_FDB;
    yaw_config.position_pid_fdb = &(obj->imu->data.yaw_8192_real);  // 陀螺仪模式反馈值更新 需参照C板实际安装方向 此处使用陀螺仪yaw轴Z
    yaw_config.speed_fdb_model = OTHER_FDB;
    yaw_config.speed_pid_fdb = &(obj->imu->data.gyro[2]);
    yaw_config.output_model = MOTOR_OUTPUT_NORMAL;
    yaw_config.lost_callback = gimbal_motor_lost;
    PID_SetConfig(&yaw_config.config_position, 1.65, 0.014, 0, 3200, 5000);
    PID_SetConfig(&yaw_config.config_speed, 235, 3, 10, 2000, 22000);
    obj->yaw = Can_Motor_Create(&yaw_config);
    can_motor_config pitch_config;
    pitch_config.motor_model = MODEL_6020;
    pitch_config.bsp_can_index = 0;
    pitch_config.motor_set_id = 1;
    pitch_config.motor_pid_model = POSITION_LOOP;
    pitch_config.position_fdb_model = OTHER_FDB;
    pitch_config.position_pid_fdb = &(obj->imu->data.euler_8192[0]);  // 此处使用陀螺仪pitch轴X
    pitch_config.speed_fdb_model = OTHER_FDB;
    pitch_config.speed_pid_fdb = &(obj->imu->data.gyro[0]);
    pitch_config.output_model = MOTOR_OUTPUT_REVERSE;
    pitch_config.lost_callback = gimbal_motor_lost;
    PID_SetConfig(&pitch_config.config_position, 1.2, 0.002, 1.1, 2500, 5000);
    PID_SetConfig(&pitch_config.config_speed, 210, 1.4, 80, 2500, 22000);
    obj->pitch = Can_Motor_Create(&pitch_config);

    // 定义sub
    obj->gimbal_cmd_sub = register_sub(gimbal_cmd_topic, 1);
    // 定义pub
    obj->gimbal_upload_pub = register_pub(gimbal_upload_topic);
    return obj;
}

void Gimbal_Update(Gimbal *gimbal) {
    // 取得控制参数
    publish_data gimbal_data = gimbal->gimbal_cmd_sub->getdata(gimbal->gimbal_cmd_sub);
    if (gimbal_data.len == -1) return;  // cmd未工作
    Gimbal_param *param = (Gimbal_param *)gimbal_data.data;

    // 重要外设掉线检测
    if ((gimbal->imu->monitor->count < 1) || !(gimbal->imu->bias_init_success)) {
        param->mode = gimbal_stop;
        gimbal->gimbal_upload_data.gimbal_module_status = module_lost;
    } else {
        gimbal->gimbal_upload_data.gimbal_module_status = module_working;
    }

    // 反馈yaw编码器信息以及云台imu是否正常工作
    publish_data gimbal_uplode;
    gimbal->gimbal_upload_data.yaw_encorder = gimbal->yaw->fdbPosition;
    gimbal_uplode.data = &(gimbal->gimbal_upload_data);
    gimbal_uplode.len = sizeof(Gimbal_uplode_data);
    gimbal->gimbal_upload_pub->publish(gimbal->gimbal_upload_pub, gimbal_uplode);

    // 模块控制
    switch (param->mode) {
        case gimbal_stop:
            gimbal->pitch->enable = MOTOR_STOP;
            gimbal->yaw->enable = MOTOR_STOP;
            break;
        case gimbal_run:
            // 启动电机
            gimbal->pitch->enable = MOTOR_ENABLE;
            gimbal->yaw->enable = MOTOR_ENABLE;
            // 设定陀螺仪控制
            gimbal->pitch->config.speed_fdb_model = OTHER_FDB;
            gimbal->pitch->config.position_fdb_model = OTHER_FDB;
            gimbal->yaw->position_pid.ref = param->yaw;
            gimbal->pitch->position_pid.ref = param->pitch;
            break;
        default:
            break;
    }

    // // 软件限位 使用编码器对pitch轴进行限位 待测
    // if ((gimbal->yaw->fdbPosition < PITCH_ENCORDER_LOWEST) || (gimbal->yaw->fdbPosition > PITCH_ENCORDER_HIGHEST))
    // {
    //     gimbal->pitch->config.speed_fdb_model = MOTOR_FDB;
    //     gimbal->pitch->config.position_fdb_model = MOTOR_FDB;
    //     if (gimbal->pitch->fdbPosition < PITCH_ENCORDER_LOWEST) gimbal->pitch->position_pid.ref = PITCH_ENCORDER_LOWEST;
    //     if (gimbal->pitch->fdbPosition > PITCH_ENCORDER_HIGHEST) gimbal->pitch->position_pid.ref = PITCH_ENCORDER_HIGHEST;
    // }
}