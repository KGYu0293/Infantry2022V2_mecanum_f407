#include "chassis.h"

#include <arm_math.h>
#include <math.h>

#include "bsp.h"
#include "common.h"

void chassis_motor_lost(void *motor) { printf_log("chassis motor lost!\n"); }
void chassis_imu_lost(void *motor) { printf_log("chassis IMU lost!!\n"); }
void chassis_super_cap_lost(void *motor) { printf_log("super cap lost!!\n"); }
void Speed_set(Chassis *obj, Cmd_chassis *param);

Chassis *Chassis_Create() {
    Chassis *obj = (Chassis *)malloc(sizeof(Chassis));
    memset(obj, 0, sizeof(Chassis));

    obj->offset_x = CHASSIS_ROTATE_X_OFFSET;
    obj->offset_y = CHASSIS_ROTATE_Y_OFFSET;

    // 外设初始化
    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 45.0f;  //设定温度为x度
    internal_imu_config.lost_callback = chassis_imu_lost;
    internal_imu_config.imu_axis_convert[0] = 1;
    internal_imu_config.imu_axis_convert[1] = 2;
    internal_imu_config.imu_axis_convert[2] = 3;
    obj->imu = BMI088_Create(&internal_imu_config);

    // 超级电容
    super_cap_wuli_config cap_config;
    cap_config.bsp_can_index = 0;
    cap_config.super_cap_wuli_rx_id = SUPER_CAP_WULI_RX_ID;
    cap_config.super_cap_wuli_tx_id = SUPER_CAP_WULI_TX_ID;
    cap_config.lost_callback = chassis_super_cap_lost;
    obj->super_cap = Super_cap_wuli_Create(&cap_config);

    can_motor_config lf_config;
    can_motor_config rf_config;
    can_motor_config lb_config;
    can_motor_config rb_config;
    controller_config lf_controller_config;
    controller_config rf_controller_config;
    controller_config lb_controller_config;
    controller_config rb_controller_config;

    // lf
    // lf_controller_config.control_type = PID_MODEL;
    lf_controller_config.control_type = PID_MODEL;
    lf_controller_config.control_depth = SPEED_CONTROL;
    PID_SetConfig_Pos(&lf_controller_config.position_pid_config, 0, 0, 0, 0, 0);
    PID_SetConfig_Pos(&lf_controller_config.speed_pid_config, 5, 0, 10, 0, 12000);
    lf_config.motor_model = MODEL_3508;
    lf_config.bsp_can_index = 0;
    lf_config.motor_set_id = 2;
    lf_config.motor_controller_config = lf_controller_config;
    lf_config.position_fdb_model = MOTOR_FDB;
    lf_config.speed_fdb_model = MOTOR_FDB;
    lf_config.output_model = MOTOR_OUTPUT_NORMAL;
    lf_config.lost_callback = chassis_motor_lost;
    obj->lf = Can_Motor_Create(&lf_config);

    // rf
    // rf_controller_config.control_type = PID_MODEL;
    rf_controller_config.control_type = PID_MODEL;
    rf_controller_config.control_depth = SPEED_CONTROL;
    PID_SetConfig_Pos(&rf_controller_config.position_pid_config, 0, 0, 0, 0, 0);
    PID_SetConfig_Pos(&rf_controller_config.speed_pid_config, 5, 0, 10, 0, 12000);
    rf_config.motor_model = MODEL_3508;
    rf_config.bsp_can_index = 0;
    rf_config.motor_set_id = 1;
    rf_config.motor_controller_config = rf_controller_config;
    rf_config.position_fdb_model = MOTOR_FDB;
    rf_config.speed_fdb_model = MOTOR_FDB;
    rf_config.output_model = MOTOR_OUTPUT_NORMAL;
    rf_config.lost_callback = chassis_motor_lost;
    obj->rf = Can_Motor_Create(&rf_config);

    // lb
    lb_controller_config.control_type = PID_MODEL;
    lb_controller_config.control_depth = SPEED_CONTROL;
    PID_SetConfig_Pos(&lb_controller_config.position_pid_config, 0, 0, 0, 0, 0);
    PID_SetConfig_Pos(&lb_controller_config.speed_pid_config, 5, 0, 10, 0, 12000);
    lb_config.motor_model = MODEL_3508;
    lb_config.bsp_can_index = 0;
    lb_config.motor_set_id = 3;
    lb_config.motor_controller_config = lb_controller_config;
    lb_config.position_fdb_model = MOTOR_FDB;
    lb_config.speed_fdb_model = MOTOR_FDB;
    lb_config.output_model = MOTOR_OUTPUT_NORMAL;
    lb_config.lost_callback = chassis_motor_lost;
    obj->lb = Can_Motor_Create(&lb_config);

    // rb
    rb_controller_config.control_type = PID_MODEL;
    rb_controller_config.control_depth = SPEED_CONTROL;
    PID_SetConfig_Pos(&rb_controller_config.position_pid_config, 0, 0, 0, 0, 0);
    PID_SetConfig_Pos(&rb_controller_config.speed_pid_config, 5, 0, 10, 0, 12000);
    rb_config.motor_model = MODEL_3508;
    rb_config.bsp_can_index = 0;
    rb_config.motor_set_id = 4;
    rb_config.motor_controller_config = rb_controller_config;
    rb_config.position_fdb_model = MOTOR_FDB;
    rb_config.speed_fdb_model = MOTOR_FDB;
    rb_config.output_model = MOTOR_OUTPUT_NORMAL;
    rb_config.lost_callback = chassis_motor_lost;
    obj->rb = Can_Motor_Create(&rb_config);

    // 定义pub
    obj->chassis_imu_pub = register_pub("upload_chassis");
    // 定义subscriber
    obj->chassis_cmd_suber = register_sub("cmd_chassis", 1);

    obj->cmd_data = NULL;
    obj->upload_data.chassis_status = module_lost;
    obj->upload_data.chassis_imu = &(obj->imu->data);
    return obj;
}

// 对电流环进行限制 简易版功率限制
void OutputmaxLimit(Chassis *obj) {
    float output_limit = 0;
    if (obj->cmd_data->power.dispatch_mode == chassis_dispatch_fly) {
        if (obj->super_cap->cap_percent < 25) {  //防止快速掉电
            output_limit = 2000;
        } else {
            output_limit = 8000;
        }
    } else {
        // output_limit = 3000 + 5000 * (obj->cmd_data->power.power_limit - 30) / 90;
        switch (obj->cmd_data->power.power_limit) {
            case 45:
                output_limit = 3000;
                break;
            case 50:
                output_limit = 3200;
                break;
            case 55:
                output_limit = 3500;
                break;
            case 60:
                output_limit = 3800;
                break;
            case 70:
                output_limit = 4200;
                break;
            case 80:
                output_limit = 4700;
                break;
            case 100:
                output_limit = 5300;
                break;
            case 120:
                output_limit = 5500;
                break;
            default:
                output_limit = 3000;  // 和最小（45w）时保持一致
                break;
        }

        if (obj->cmd_data->power.dispatch_mode == chassis_dispatch_shift) {
            output_limit *= 1.25;
        }

        if (output_limit < 2000) output_limit = 2000;
        if (output_limit > 8000) output_limit = 8000;
        if (obj->super_cap->cap_percent < 30)
            output_limit = 2000;  // 1500
        else if (obj->super_cap->cap_percent < 50)
            output_limit *= 0.9;
    }

    // outputlimit按照需求进行分配
    float output_sum = fabs(obj->lf->motor_controller->output) + fabs(obj->lb->motor_controller->output) + fabs(obj->rf->motor_controller->output) + fabs(obj->rb->motor_controller->output);
    if (output_sum > 1.0f) {
        // 保证四个outputmax之和为4*output_limit
        obj->lf->motor_controller->pid_speed_data.config.outputMax = /*0.25 * output_limit + 3*/ 4 * output_limit * fabs(obj->lf->motor_controller->output) / output_sum;
        obj->rf->motor_controller->pid_speed_data.config.outputMax = /*0.25 * output_limit + 3*/ 4 * output_limit * fabs(obj->rf->motor_controller->output) / output_sum;
        obj->lb->motor_controller->pid_speed_data.config.outputMax = /*0.25 * output_limit + 3*/ 4 * output_limit * fabs(obj->lb->motor_controller->output) / output_sum;
        obj->rb->motor_controller->pid_speed_data.config.outputMax = /*0.25 * output_limit + 3*/ 4 * output_limit * fabs(obj->rb->motor_controller->output) / output_sum;
    } else {
        obj->lf->motor_controller->pid_speed_data.config.outputMax = output_limit;
        obj->rf->motor_controller->pid_speed_data.config.outputMax = output_limit;
        obj->lb->motor_controller->pid_speed_data.config.outputMax = output_limit;
        obj->rb->motor_controller->pid_speed_data.config.outputMax = output_limit;
    }
}

/**
 * @brief 底盘电机缓启动
 * @param None
 * @retval None
 */
void ChassisAccelerationLimit(Chassis *obj) {
    // double accMax = 15.0f * (double)param->power.power_limit;
    // if (accMax < 170.0f)     accMax = 170.0f;
    // else if (accMax > 270.0f)    accMax = 270.0f;

    // 功率控制良好的情况下acc limit主要防打滑 不必与功率相关
    float accMax = 1500;  // 1020-1620
    if (fabs(obj->lf->motor_controller->ref_speed - obj->lf->motor_controller->fdb_speed) > accMax) {
        obj->lf->motor_controller->ref_speed = obj->lf->motor_controller->fdb_speed + accMax * (obj->lf->motor_controller->ref_speed - obj->lf->motor_controller->fdb_speed > 0 ? 1 : -1);
    }
    if (fabs(obj->lb->motor_controller->ref_speed - obj->lb->motor_controller->fdb_speed) > accMax) {
        obj->lb->motor_controller->ref_speed = obj->lb->motor_controller->fdb_speed + accMax * (obj->lb->motor_controller->ref_speed - obj->lb->motor_controller->fdb_speed > 0 ? 1 : -1);
    }
    if (fabs(obj->rf->motor_controller->ref_speed - obj->rf->motor_controller->fdb_speed) > accMax) {
        obj->rf->motor_controller->ref_speed = obj->rf->motor_controller->fdb_speed + accMax * (obj->rf->motor_controller->ref_speed - obj->rf->motor_controller->fdb_speed > 0 ? 1 : -1);
    }
    if (fabs(obj->rb->motor_controller->ref_speed - obj->rb->motor_controller->fdb_speed) > accMax) {
        obj->rb->motor_controller->ref_speed = obj->rb->motor_controller->fdb_speed + accMax * (obj->rb->motor_controller->ref_speed - obj->rb->motor_controller->fdb_speed > 0 ? 1 : -1);
    }
}

/**
 * @brief 正常运行模式下，麦轮底盘的解算函数
 * @param X方向的速度（向右为正，mm/s）,Y方向的速度（向前为正，mm/s）
 */
void mecanum_calculate(Chassis *obj, float vx, float vy, float rotate) {
    float r_x, r_y;
    float mecanum_speed[4];

    r_x = CHASSIS_WHEELTRACK / 2 + obj->offset_x;
    r_y = CHASSIS_WHEELBASE / 2 - obj->offset_y;
    mecanum_speed[0] = vx + vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = CHASSIS_WHEELTRACK / 2 - obj->offset_x;
    r_y = CHASSIS_WHEELBASE / 2 - obj->offset_y;
    mecanum_speed[1] = vx - vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = CHASSIS_WHEELTRACK / 2 - obj->offset_x;
    r_y = CHASSIS_WHEELBASE / 2 + obj->offset_y;
    mecanum_speed[2] = -vx - vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = CHASSIS_WHEELTRACK / 2 + obj->offset_x;
    r_y = CHASSIS_WHEELBASE / 2 + obj->offset_y;
    mecanum_speed[3] = -vx + vy - rotate * (r_x + r_y) / RADIAN_COEF;

    obj->lf->motor_controller->ref_speed = mecanum_speed[0] / CHASSIS_PERIMETER * CHASSIS_MOTOR_DECELE_RATIO * 360;  // rpm: *60  度/s: /360
    obj->rf->motor_controller->ref_speed = mecanum_speed[1] / CHASSIS_PERIMETER * CHASSIS_MOTOR_DECELE_RATIO * 360;
    obj->rb->motor_controller->ref_speed = mecanum_speed[2] / CHASSIS_PERIMETER * CHASSIS_MOTOR_DECELE_RATIO * 360;
    obj->lb->motor_controller->ref_speed = mecanum_speed[3] / CHASSIS_PERIMETER * CHASSIS_MOTOR_DECELE_RATIO * 360;
}

// 小陀螺情况下的旋转速度控制函数
float auto_rotate_param(Cmd_chassis *param) {
    static float rotate = 0;
    // 位置式变速
    float rotate_baseline = 170 + (param->power.power_limit - 45) * 2;  // 该功率下的基准转速 线性拟合
    // 还是单独测吧（
    switch (param->power.power_limit) {
        case 45:
            rotate_baseline = 250;
            break;
        case 50:
            rotate_baseline = 260;
            break;
        case 55:
            rotate_baseline = 280;
            break;
        case 60:
            rotate_baseline = 310;
            break;
        case 70:
            rotate_baseline = 330;
            break;
        case 80:
            rotate_baseline = 350;
            break;
        case 100:
            rotate_baseline = 350;
            break;
        case 120:
            rotate_baseline = 360;
            break;
        default:
            rotate_baseline = 240;  // 和最小（45w）时保持一致
            break;
    }

    // float x = (param->target.offset_angle / RADIAN_COEF) - 0.25 * pi;  // 原点 换算成弧度 加定值使速度最低时装甲板不在正面
    // rotate = rotate_baseline + rotate_baseline * 0.2 * sin(x);       // 变速函数&变速范围
    rotate = rotate_baseline;
    // 时间式变速
    // static uint8_t spin_speed_change = 1;// 0：定速 1：加速 2：减速 （初始从低往高加）
    // // 基准转速 = 最低转速 + 高功率下加速旋转
    // float rotate_benchmark = 150 + (param->power.power_limit - 45) * 2;
    // if (spin_speed_change == 0) {
    //     // 定速
    //     rotate = rotate_benchmark;
    // } else {
    //     // 变速范围
    //     float rotate_max = rotate_benchmark + 50;
    //     float rotate_min = rotate_benchmark - 50;
    //     if (rotate < rotate_min) rotate = rotate_min;
    //     if (rotate > rotate_max) rotate = rotate_max;
    //     if (spin_speed_change == 1) {
    //         // 加速（目前为线性）
    //         rotate += 0.0005;
    //         if (rotate > rotate_max){
    //             spin_speed_change = 2;
    //         }
    //     }else {
    //         // 减速 建议不要使用功率减速 以使同功率下整体平均转速较高
    //         rotate -= 0.0001;
    //         if (rotate < rotate_min){
    //             spin_speed_change = 1;
    //         }
    //     }
    // }

    // rotate = 150;
    rotate *= 0.9;
    return rotate;
}

// 将基于offset的速度映射到实际底盘坐标系的方向上
void Chassis_calculate(Chassis *obj) {
    // 基准直线速度
    obj->proc_v_base = 1500 + ((float)obj->cmd_data->power.power_limit - 45) * 60;
    switch (obj->cmd_data->power.power_limit) {
        case 45:
            obj->proc_v_base = 2700;
            break;
        case 50:
            obj->proc_v_base = 3000;
            break;
        case 55:
            obj->proc_v_base = 3200;
            break;
        case 60:
            obj->proc_v_base = 3500;
            break;
        case 70:
            obj->proc_v_base = 3700;
            break;
        case 80:
            obj->proc_v_base = 3900;
            break;
        case 100:
            obj->proc_v_base = 4000;
            break;
        case 120:
            obj->proc_v_base = 4000;
            break;
        default:
            obj->proc_v_base = 2700;  // 和最小（45w）时保持一致
            break;
    }

    obj->proc_v_base *= 0.8;  // 调试 降功率

    // //同时按住前后和平移
    // if (fabs(obj->cmd_data->target.vx) > 1e-5 && fabs(obj->cmd_data->target.vy) > 1e-5) {
    //     obj->cmd_data->target.vx *= 0.6;  //平移减速
    // }
    float a = ((obj->cmd_data->target.vx * obj->cmd_data->target.vx) + (obj->cmd_data->target.vy * obj->cmd_data->target.vy));
    float ratio;
    arm_sqrt_f32(a, &ratio);                               // 使用armmath库代替c语言库的sqrt加快速度
    if (ratio > 4) ratio = 4;                              // 最大爆发速度倍率限制
    obj->proc_v_base = obj->proc_v_base * ratio;           // 理论上应该取cmd_data->target.vx/y绝对值中较大的一个
    if (obj->proc_v_base > 5000) obj->proc_v_base = 5000;  // 最大上限设置值
    if (obj->cmd_data->power.dispatch_mode == chassis_dispatch_fly) {
        obj->proc_v_base = 5000;  // 飞坡模式速度设定 5m/s
    }

    if (fabs(ratio) < 1e-5) {
        obj->proc_target_vx = obj->proc_target_vy = 0;
    } else {
        obj->proc_target_vx = obj->proc_v_base * obj->cmd_data->target.vx / ratio;
        obj->proc_target_vy = obj->proc_v_base * obj->cmd_data->target.vy / ratio;
    }

    // 计算旋转速度
    float w = obj->cmd_data->target.rotate;
    if (obj->cmd_data->mode == chassis_rotate_run) {
        w = auto_rotate_param(obj->cmd_data);
    } else if (obj->cmd_data->mode == chassis_run_follow_offset) {
        // 平方根函数
        arm_sqrt_f32(2000 * fabs(obj->cmd_data->target.offset_angle), &w);
        w *= sgn(obj->cmd_data->target.offset_angle);
        // 采用二次函数
        // w = 0.20f * (obj->cmd_data->target.offset_angle) * fabs(obj->cmd_data->target.offset_angle);
        //飞坡模式要求底盘跟随云台更加紧密
        if (obj->cmd_data->power.dispatch_mode == chassis_dispatch_fly) {
            w *= 1.8;
        }
    }

    //小陀螺加速
    if (obj->cmd_data->power.dispatch_mode == chassis_dispatch_shift && obj->cmd_data->mode == chassis_rotate_run) {
        w *= 1.25;
    }

    // 边旋转边平移的功率分配
    if (obj->cmd_data->mode == chassis_rotate_run && fabs(ratio) > 1e-5) {
        obj->proc_target_vx *= 0.3;
        obj->proc_target_vy *= 0.3;
        w *= 0.7;
    }

    // 麦轮解算
    float chassis_vx = obj->proc_target_vx * cos(obj->cmd_data->target.offset_angle * DEG2RAD) - obj->proc_target_vy * sin(obj->cmd_data->target.offset_angle * DEG2RAD);
    float chassis_vy = obj->proc_target_vx * sin(obj->cmd_data->target.offset_angle * DEG2RAD) + obj->proc_target_vy * cos(obj->cmd_data->target.offset_angle * DEG2RAD);
    mecanum_calculate(obj, chassis_vx, chassis_vy, w);
    // 加速度限制
    if (obj->cmd_data->power.dispatch_mode == chassis_dispatch_mild) {
        ChassisAccelerationLimit(obj);
    }
    // 功率控制
    OutputmaxLimit(obj);
}

void Chassis_Update(Chassis *obj) {
    // 检查imu在线并初始化完成
    if (obj->imu->monitor->count < 1 || !(obj->imu->bias_init_success)) {
        obj->upload_data.chassis_status = module_lost;
        obj->cmd_data->mode = robot_stop;
    } else {
        obj->upload_data.chassis_status = module_working;
    }

    // 电容剩余值
    obj->upload_data.chassis_supercap_percent = obj->super_cap->cap_percent;
    obj->upload_data.chassis_battery_voltage = obj->super_cap->voltage_input_fdb;
    // 发送回传数据指针
    publish_data chassis_upload;
    chassis_upload.data = (uint8_t *)&(obj->upload_data);
    chassis_upload.len = sizeof(Upload_chassis);
    obj->chassis_imu_pub->publish(obj->chassis_imu_pub, chassis_upload);

    // 设置电容充电功率，在缓冲功率充足时，多充能
    // stop模式等各种模式要能执行这一段，否则可能会超功率
    if (obj->cmd_data->power.power_buffer > 60) {  // 按照目前规则 仅触发飞坡增益时超过60（为250）
        obj->super_cap->power_set = obj->cmd_data->power.power_limit + 10;
    } else if (obj->cmd_data->power.power_buffer > 30) {
        obj->super_cap->power_set = obj->cmd_data->power.power_limit + 10;
    } else {
        obj->super_cap->power_set = obj->cmd_data->power.power_limit - 2;  // 防止电容超功率
    }

    // 获得cmd命令
    publish_data chassis_data = obj->chassis_cmd_suber->getdata(obj->chassis_cmd_suber);
    if (chassis_data.len == -1) return;  // 未收到指令
    obj->cmd_data = (Cmd_chassis *)chassis_data.data;

    // 应用得到的param进行控制
    if (obj->cmd_data->mode == chassis_stop) {
        obj->lf->enable = MOTOR_STOP;
        obj->lb->enable = MOTOR_STOP;
        obj->rf->enable = MOTOR_STOP;
        obj->rb->enable = MOTOR_STOP;
    } else {
        obj->lf->enable = MOTOR_ENABLE;
        obj->lb->enable = MOTOR_ENABLE;
        obj->rf->enable = MOTOR_ENABLE;
        obj->rb->enable = MOTOR_ENABLE;
        Chassis_calculate(obj);
    }
}
