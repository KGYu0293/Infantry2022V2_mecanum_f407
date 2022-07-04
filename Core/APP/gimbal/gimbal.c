#include "gimbal.h"

#include "bsp_def.h"

void gimbal_motor_lost(void *motor) { printf_log("gimbal motor lost!\n"); }
void gimbal_imu_lost(void *imu) { printf_log("gimbal imu lost!!!gimbal stopped.\n"); }

Gimbal *Gimbal_Create() {
    Gimbal *obj = (Gimbal *)malloc(sizeof(Gimbal));
    memset(obj, 0, sizeof(Gimbal));

    //外设初始化
    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 45.0f;  //设定温度为45度
    internal_imu_config.lost_callback = gimbal_imu_lost;
    //定义转换矩阵
    internal_imu_config.imu_axis_convert[0] = 2;
    internal_imu_config.imu_axis_convert[1] = 3;
    internal_imu_config.imu_axis_convert[2] = 1;
    obj->imu = BMI088_Create(&internal_imu_config);
    
    controller_config yaw_controller_config;
    yaw_controller_config.control_type = PID_MODEL;
    yaw_controller_config.control_depth = POS_CONTROL;
    PID_SetConfig_Pos(&yaw_controller_config.position_pid_config, 17.6, 0.1, 6, 141, 5000);
    PID_SetConfig_Pos(&yaw_controller_config.speed_pid_config, 235, 3, 10, 2000, 25000);
    can_motor_config yaw_config;
    yaw_config.motor_controller_config = yaw_controller_config;
    yaw_config.motor_model = MODEL_6020;
    yaw_config.bsp_can_index = 1;
    yaw_config.motor_set_id = 1;
    yaw_config.position_fdb_model = OTHER_FDB;
    yaw_config.position_fdb = &(obj->imu->data.yaw_deg_real);  // 此处使用陀螺仪yaw轴Z
    yaw_config.speed_fdb_model = OTHER_FDB;
    yaw_config.speed_fdb = &(obj->imu->data.gyro_deg[YAW_AXIS]);
    yaw_config.output_model = MOTOR_OUTPUT_NORMAL;
    yaw_config.lost_callback = gimbal_motor_lost;
    obj->yaw = Can_Motor_Create(&yaw_config);

    controller_config pitch_controller_config;
    pitch_controller_config.control_type = PID_MODEL;
    pitch_controller_config.control_depth = POS_CONTROL;
    PID_SetConfig_Pos(&pitch_controller_config.position_pid_config, 31.86, 0.068, 36.41, 109.9, 5000);
    PID_SetConfig_Pos(&pitch_controller_config.speed_pid_config, 190, 0.7, 1, 5000, 25000);
    can_motor_config pitch_config;
    pitch_config.motor_model = MODEL_6020;
    pitch_config.bsp_can_index = 0;
    pitch_config.motor_set_id = 1;
    pitch_config.motor_controller_config = pitch_controller_config;
    pitch_config.position_fdb_model = OTHER_FDB;
    pitch_config.position_fdb = &(obj->imu->data.euler_deg[PITCH_AXIS]);  // 此处使用陀螺仪pitch轴X
    pitch_config.speed_fdb_model = OTHER_FDB;
    pitch_config.speed_fdb = &(obj->imu->data.gyro_deg[PITCH_AXIS]);
    pitch_config.output_model = MOTOR_OUTPUT_REVERSE;
    pitch_config.lost_callback = gimbal_motor_lost;
    obj->pitch = Can_Motor_Create(&pitch_config);

    // 定义sub、pub
    obj->gimbal_cmd_sub = register_sub("cmd_gimbal", 1);
    obj->gimbal_upload_pub = register_pub("upload_gimbal");
    memset(&(obj->gimbal_upload_data), 0, sizeof(Upload_gimbal));
    obj->cmd_data = NULL;

    return obj;
}

void Gimbal_Update(Gimbal *gimbal) {
    // 取得控制参数
    publish_data gimbal_data = gimbal->gimbal_cmd_sub->getdata(gimbal->gimbal_cmd_sub);
    if (gimbal_data.len == -1) return;  // cmd未工作
    gimbal->cmd_data = (Cmd_gimbal *)gimbal_data.data;

    // 重要外设掉线检测
    if ((gimbal->imu->monitor->count < 1) || !(gimbal->imu->bias_init_success)) {
        gimbal->cmd_data->mode = gimbal_stop;
        gimbal->gimbal_upload_data.gimbal_status = module_lost;
    } else {
        gimbal->gimbal_upload_data.gimbal_status = module_working;
    }

    // 反馈yaw编码器信息以及云台imu是否正常工作
    publish_data gimbal_upload;
    gimbal->gimbal_upload_data.yaw_encorder = &(gimbal->yaw->fdbPosition);
    gimbal->gimbal_upload_data.gimbal_imu = &(gimbal->imu->data);
    gimbal_upload.data = (uint8_t *)&(gimbal->gimbal_upload_data);
    gimbal_upload.len = sizeof(Upload_gimbal);
    gimbal->gimbal_upload_pub->publish(gimbal->gimbal_upload_pub, gimbal_upload);

    // p轴限位值获取
    gimbal->pitch_limit_down = 0.2f * gimbal->pitch_limit_down + 0.8f * (gimbal->imu->data.euler_deg[1] + (gimbal->pitch->fdbPosition - PITCH_ENCORDER_LOWEST) * 360.0 / 8192);
    gimbal->pitch_limit_up = 0.2f * gimbal->pitch_limit_up + 0.8f * (gimbal->imu->data.euler_deg[1] - (PITCH_ENCORDER_HIGHEST - gimbal->pitch->fdbPosition) * 360.0 / 8192);

    // 模块控制
    switch (gimbal->cmd_data->mode) {
        case gimbal_stop:
            gimbal->yaw->enable = MOTOR_STOP;
            gimbal->pitch->enable = MOTOR_STOP;
            break;
        case gimbal_run:
            // 启动电机
            gimbal->yaw->enable = MOTOR_ENABLE;
            gimbal->pitch->enable = MOTOR_ENABLE;
            // 设定陀螺仪控制
            // PID_SetConfig(&gimbal->pitch->position_pid.config, 1.4, 0.003, 1.6, 2500, 5000);
            // PID_SetConfig(&gimbal->pitch->speed_pid.config, 170, 0.7, 1, 5000, 25000);
            gimbal->yaw->config.speed_fdb_model = OTHER_FDB;
            gimbal->yaw->config.position_fdb_model = OTHER_FDB;
            gimbal->pitch->config.speed_fdb_model = OTHER_FDB;
            gimbal->pitch->config.speed_fdb = &(gimbal->imu->data.gyro_deg[PITCH_AXIS]);
            gimbal->pitch->config.position_fdb_model = OTHER_FDB;
            gimbal->pitch->config.output_model = MOTOR_OUTPUT_REVERSE;
            // yaw轴
            gimbal->yaw->motor_controller->ref_position = gimbal->cmd_data->yaw;
            // pitch轴
            gimbal->pitch->motor_controller->ref_position = gimbal->cmd_data->pitch;
            // p轴软件限位
            if (gimbal->pitch->motor_controller->ref_position < gimbal->pitch_limit_up) gimbal->pitch->motor_controller->ref_position = gimbal->pitch_limit_up;
            if (gimbal->pitch->motor_controller->ref_position > gimbal->pitch_limit_down) gimbal->pitch->motor_controller->ref_position = gimbal->pitch_limit_down;
            break;
        // 跟随底盘
        case gimbal_middle:
            // 启动电机
            gimbal->yaw->enable = MOTOR_ENABLE;
            gimbal->pitch->enable = MOTOR_ENABLE;
            // 设定编码器控制
            // PID_SetConfig(&gimbal->pitch->position_pid.config, 0.1, 0.003, 0, 2500, 5000);
            // PID_SetConfig(&gimbal->pitch->speed_pid.config, 30, 0.7, 0, 5000, 25000);
            // gimbal->yaw->config.speed_fdb_model = MOTOR_FDB;
            gimbal->yaw->config.position_fdb_model = MOTOR_FDB;
            // gimbal->pitch->config.speed_fdb_model = MOTOR_FDB;
            gimbal->pitch->config.position_fdb_model = MOTOR_FDB;
            gimbal->pitch->config.speed_fdb = &(gimbal->imu->data.gyro_deg_reverse[PITCH_AXIS]);
            //将编码器值remap到0-360
            gimbal->yaw->motor_controller->ref_position = YAW_MOTOR_ENCORDER_BIAS * 360.0 / 8192 + gimbal->yaw->round * 360;
            gimbal->pitch->motor_controller->ref_position = PITCH_MOTOR_ENCORDER_BIAS * 360.0 / 8192;
            gimbal->pitch->config.output_model = MOTOR_OUTPUT_NORMAL;
            break;
    }
}