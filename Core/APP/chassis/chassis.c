#include "chassis.h"

#include <math.h>

#include "bsp.h"

// the radius of wheel(mm)，轮子半径
#define RADIUS 71  // 71.25

// the perimeter of wheel(mm)，轮子周长
#define PERIMETER 448  // 71.25*2pi

// wheel track distance(mm)，轮距
#define WHEELTRACK 340

// wheelbase distance(mm)，轴距
#define WHEELBASE 340

// gimbal is relative to chassis center x axis offset(mm)，云台相对于底盘中心的偏移，往右为正
#define ROTATE_X_OFFSET 0

// gimbal is relative to chassis center y axis offset(mm)，云台相对于底盘中心的偏移，往前为正
#define ROTATE_Y_OFFSET 0

// the deceleration ratio of chassis motor，底盘电机减速比
#define MOTOR_DECELE_RATIO 19.0f

#define RADIAN_COEF 57.3f  // 180°/pi

void chassis_motor_lost(void *motor) { printf_log("chassis motor lost!\n"); }
void chassis_imu_lost(void *motor) { printf_log("chassis IMU lost!!\n"); }
void chassis_super_cap_lost(void *motor) { printf_log("super cap lost!!\n"); }
void Speed_set(Chassis *obj, Cmd_chassis *param);

Chassis *Chassis_Create() {
    Chassis *obj = (Chassis *)malloc(sizeof(Chassis));
    memset(obj, 0, sizeof(Chassis));

    obj->offset_x = ROTATE_X_OFFSET;
    obj->offset_y = ROTATE_Y_OFFSET;

    // 外设初始化
    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 55.0f;  //设定温度为55度
    internal_imu_config.lost_callback = chassis_imu_lost;
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
    lf_config.motor_model = MODEL_3508;
    lf_config.bsp_can_index = 0;
    lf_config.motor_set_id = 4;
    lf_config.motor_pid_model = SPEED_LOOP;
    lf_config.position_fdb_model = MOTOR_FDB;
    lf_config.speed_fdb_model = MOTOR_FDB;
    lf_config.output_model = MOTOR_OUTPUT_NORMAL;
    lf_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&lf_config.config_position, 0, 0, 0, 0, 5000);
    PID_SetConfig(&lf_config.config_speed, 5, 0, 10, 0, 10000);
    obj->lf = Can_Motor_Create(&lf_config);
    rf_config.motor_model = MODEL_3508;
    rf_config.bsp_can_index = 0;
    rf_config.motor_set_id = 3;
    rf_config.motor_pid_model = SPEED_LOOP;
    rf_config.position_fdb_model = MOTOR_FDB;
    rf_config.speed_fdb_model = MOTOR_FDB;
    rf_config.output_model = MOTOR_OUTPUT_NORMAL;
    rf_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&rf_config.config_position, 0, 0, 0, 0, 5000);
    PID_SetConfig(&rf_config.config_speed, 5, 0, 10, 0, 10000);
    obj->rf = Can_Motor_Create(&rf_config);
    lb_config.motor_model = MODEL_3508;
    lb_config.bsp_can_index = 0;
    lb_config.motor_set_id = 1;
    lb_config.motor_pid_model = SPEED_LOOP;
    lb_config.position_fdb_model = MOTOR_FDB;
    lb_config.speed_fdb_model = MOTOR_FDB;
    lb_config.output_model = MOTOR_OUTPUT_NORMAL;
    lb_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&lb_config.config_position, 0, 0, 0, 0, 5000);
    PID_SetConfig(&lb_config.config_speed, 5, 0, 10, 0, 10000);
    obj->lb = Can_Motor_Create(&lb_config);
    rb_config.motor_model = MODEL_3508;
    rb_config.bsp_can_index = 0;
    rb_config.motor_set_id = 2;
    rb_config.motor_pid_model = SPEED_LOOP;
    rb_config.position_fdb_model = MOTOR_FDB;
    rb_config.speed_fdb_model = MOTOR_FDB;
    rb_config.output_model = MOTOR_OUTPUT_NORMAL;
    rb_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&rb_config.config_position, 0, 0, 0, 0, 5000);
    PID_SetConfig(&rb_config.config_speed, 5, 0, 10, 0, 10000);
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

//对电流环进行限制 功率限制
void OutputmaxLimit(Chassis *obj) {
    float output_limit = 0;
    if (obj->cmd_data->power.if_consume_supercap) {
        if (obj->super_cap->cap_percent < 30) {
            output_limit = 2000;
        } else if (obj->super_cap->cap_percent < 50) {
            output_limit = 5000;
        } else {
            output_limit = 8000;
        }
    } else {
        output_limit = 3000 + 5000 * (obj->cmd_data->power.power_limit - 30) / 90;
        if (output_limit < 3000) output_limit = 3000;
        if (output_limit > 8000) output_limit = 8000;
        if (obj->super_cap->cap_percent < 30) output_limit = 2000;
    }
    obj->lf->speed_pid.config.outputMax = output_limit;
    obj->rf->speed_pid.config.outputMax = output_limit;
    obj->lb->speed_pid.config.outputMax = output_limit;
    obj->rb->speed_pid.config.outputMax = output_limit;
}

/**
 * @brief 正常运行模式下，麦轮底盘的解算函数
 * @param X方向的速度（向右为正，mm/s）,Y方向的速度（向前为正，mm/s）
 */
void mecanum_calculate(Chassis *obj, float vx, float vy, float rotate) {
    float r_x, r_y;
    float mecanum_speed[4];

    r_x = WHEELTRACK / 2 + obj->offset_x;
    r_y = WHEELBASE / 2 - obj->offset_y;
    mecanum_speed[0] = -vx - vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = WHEELTRACK / 2 - obj->offset_x;
    r_y = WHEELBASE / 2 - obj->offset_y;
    mecanum_speed[1] = -vx + vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = WHEELTRACK / 2 - obj->offset_x;
    r_y = WHEELBASE / 2 + obj->offset_y;
    mecanum_speed[2] = vx + vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = WHEELTRACK / 2 + obj->offset_x;
    r_y = WHEELBASE / 2 + obj->offset_y;
    mecanum_speed[3] = vx - vy - rotate * (r_x + r_y) / RADIAN_COEF;

    obj->lf->speed_pid.ref = mecanum_speed[0] / PERIMETER * MOTOR_DECELE_RATIO * 360;  // rpm: *60  度/s: /360
    obj->rf->speed_pid.ref = mecanum_speed[1] / PERIMETER * MOTOR_DECELE_RATIO * 360;
    obj->rb->speed_pid.ref = mecanum_speed[2] / PERIMETER * MOTOR_DECELE_RATIO * 360;
    obj->lb->speed_pid.ref = mecanum_speed[3] / PERIMETER * MOTOR_DECELE_RATIO * 360;
}

// 小陀螺情况下的旋转速度控制函数，可以写不同的变速小陀螺
float auto_rotate_param(void) { return 150; }

// 将基于offset的速度映射到实际底盘坐标系的方向上
void Chassis_calculate(Chassis *obj, Cmd_chassis *param) {
    //功率控制
    OutputmaxLimit(obj);

    float vx = param->target.vx * cos(param->target.offset_angle * DEG2RAD) - param->target.vy * sin(param->target.offset_angle * DEG2RAD);
    float vy = param->target.vx * sin(param->target.offset_angle * DEG2RAD) + param->target.vy * cos(param->target.offset_angle * DEG2RAD);
    if (param->mode == chassis_run)
        mecanum_calculate(obj, vx, vy, param->target.rotate);
    else if (param->mode == chassis_rotate_run) {
        float w = auto_rotate_param();
        mecanum_calculate(obj, vx, vy, w);
    } else if (param->mode == chassis_run_follow_offset) {
        float w = 0.1f * (param->target.offset_angle) * fabs(param->target.offset_angle);  // 采用二次函数
        mecanum_calculate(obj, vx, vy, w);
    }
}

void Chassis_Update(Chassis *obj) {
    // 检查imu在线并初始化完成
    if (obj->imu->monitor->count < 1 || !(obj->imu->bias_init_success)) {
        obj->upload_data.chassis_status = module_lost;
    } else {
        obj->upload_data.chassis_status = module_working;
    }

    //电容剩余值
    obj->upload_data.chassis_supercap_percent = obj->super_cap->cap_percent;
    // 发送回传数据指针
    publish_data chassis_upload;
    chassis_upload.data = (uint8_t *)&(obj->upload_data);
    chassis_upload.len = sizeof(Upload_chassis);
    obj->chassis_imu_pub->publish(obj->chassis_imu_pub, chassis_upload);

    // 获得cmd命令
    publish_data chassis_data = obj->chassis_cmd_suber->getdata(obj->chassis_cmd_suber);
    if (chassis_data.len == -1) return;  // 未收到指令
    obj->cmd_data = (Cmd_chassis *)chassis_data.data;

    //设置电容充电功率，在缓冲功率充足时，多充能
    if (obj->cmd_data->power.power_buffer > 30) {
        obj->super_cap->power_set = obj->cmd_data->power.power_limit + 10;
    } else {
        obj->super_cap->power_set = obj->cmd_data->power.power_limit;
    }

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
        Chassis_calculate(obj, obj->cmd_data);
    }
}
