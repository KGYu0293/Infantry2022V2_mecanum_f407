#include "chassis.h"

#include <math.h>

/* the radius of wheel(mm)，轮子半径 */
#define RADIUS 71  // 71.25
/* the perimeter of wheel(mm)，轮子周长 */
#define PERIMETER 448  // 71.25*2pi
/* wheel track distance(mm)，轮距 */
#define WHEELTRACK 340
/* wheelbase distance(mm)，轴距 */
#define WHEELBASE 340
/* gimbal is relative to chassis center x axis offset(mm)，云台相对于底盘中心的偏移，往右为正 */
#define ROTATE_X_OFFSET 0
/* gimbal is relative to chassis center y axis offset(mm)，云台相对于底盘中心的偏移，往前为正 */
#define ROTATE_Y_OFFSET 0
/* the deceleration ratio of chassis motor，底盘电机减速比 */
#define MOTOR_DECELE_RATIO 19.0f

#define RADIAN_COEF 57.3f  // 180°/pi

cvector *chassis_instances;

void chassis_motor_lost(void *motor) { printf_log("chassis motor lost!\n"); }

Chassis *Chassis_Create() {
    Chassis *obj = (Chassis *)malloc(sizeof(Chassis));

    obj->offset_x = ROTATE_X_OFFSET;
    obj->offset_y = ROTATE_Y_OFFSET;

    // 电机初始化
    can_motor_config lf_config;
    can_motor_config rf_config;
    can_motor_config lb_config;
    can_motor_config rb_config;
    lf_config.motor_model = MODEL_3508;
    lf_config.bsp_can_index = 0;
    lf_config.motor_set_id = 1;
    lf_config.motor_pid_model = SPEED_LOOP;
    lf_config.position_fdb_model = MOTOR_FDB;
    lf_config.speed_fdb_model = MOTOR_FDB;
    lf_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&lf_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&lf_config.config_speed, 20, 0, 0, 2000, 12000);
    obj->lf = Can_Motor_Create(&lf_config);
    rf_config.motor_model = MODEL_3508;
    rf_config.bsp_can_index = 0;
    rf_config.motor_set_id = 2;
    rf_config.motor_pid_model = SPEED_LOOP;
    rf_config.position_fdb_model = MOTOR_FDB;
    rf_config.speed_fdb_model = MOTOR_FDB;
    rf_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&rf_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&rf_config.config_speed, 20, 0, 0, 2000, 12000);
    obj->rf = Can_Motor_Create(&rf_config);
    lb_config.motor_model = MODEL_3508;
    lb_config.bsp_can_index = 0;
    lb_config.motor_set_id = 3;
    lb_config.motor_pid_model = SPEED_LOOP;
    lb_config.position_fdb_model = MOTOR_FDB;
    lb_config.speed_fdb_model = MOTOR_FDB;
    lb_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&lb_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&lb_config.config_speed, 20, 0, 0, 2000, 12000);
    obj->lb = Can_Motor_Create(&lb_config);
    rb_config.motor_model = MODEL_3508;
    rb_config.bsp_can_index = 0;
    rb_config.motor_set_id = 40;
    rb_config.motor_pid_model = SPEED_LOOP;
    rb_config.position_fdb_model = MOTOR_FDB;
    rb_config.speed_fdb_model = MOTOR_FDB;
    rb_config.lost_callback = chassis_motor_lost;
    PID_SetConfig(&rb_config.config_position, 2, 0, 0, 0, 5000);
    PID_SetConfig(&rb_config.config_speed, 20, 0, 0, 2000, 12000);
    obj->rb = Can_Motor_Create(&rb_config);

    // 定义subscriber
    obj->chassis_cmd_suber = register_sub(chassis_cmd_topic,sizeof(Chassis_param));
    
    return obj;
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
    mecanum_speed[0] = vx - vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = WHEELTRACK / 2 - obj->offset_x;
    r_y = WHEELBASE / 2 - obj->offset_y;
    mecanum_speed[1] = vx + vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = WHEELTRACK / 2 - obj->offset_x;
    r_y = WHEELBASE / 2 + obj->offset_y;
    mecanum_speed[2] = -vx + vy - rotate * (r_x + r_y) / RADIAN_COEF;
    r_x = WHEELTRACK / 2 + obj->offset_x;
    r_y = WHEELBASE / 2 + obj->offset_y;
    mecanum_speed[3] = -vx - vy - rotate * (r_x + r_y) / RADIAN_COEF;

    obj->lf->speed_pid.ref = mecanum_speed[0] / PERIMETER * MOTOR_DECELE_RATIO / 360;  // rpm: *60  度/s: /360
    obj->rf->speed_pid.ref = mecanum_speed[1] / PERIMETER * MOTOR_DECELE_RATIO / 360;
    obj->rb->speed_pid.ref = mecanum_speed[2] / PERIMETER * MOTOR_DECELE_RATIO / 360;
    obj->lb->speed_pid.ref = mecanum_speed[3] / PERIMETER * MOTOR_DECELE_RATIO / 360;
}

// 小陀螺情况下的旋转速度控制函数，可以写不同的变速小陀螺
float auto_rotate_param(void) { return 120; }

// 将基于offset的速度映射到实际底盘坐标系的方向上
void Chassis_calculate(Chassis *obj, Chassis_param *param) {
    if (param->target.offset_angle < 0) param->target.offset_angle = 0;
    if (param->target.offset_angle > 360) param->target.offset_angle = 360;
    float vx = param->target.vx * cos(param->target.offset_angle) + param->target.vy * sin(param->target.offset_angle);
    float vy = param->target.vx * sin(param->target.offset_angle) + param->target.vy * cos(param->target.offset_angle);
    if (param->mode = chassis_run) mecanum_calculate(obj, vx, vy, param->target.rotate);
    if (param->mode = chassis_rotate_run) {
        float w = auto_rotate_param();
        mecanum_calculate(obj, vx, vy, w);
    }
    // 缓启动 斜坡
}

void Chassis_Update(Chassis *obj) {
    // subscribe并得到param
    Chassis_param param;

    switch (param.mode) {
        case chassis_stop:
        case chassis_run:
            Chassis_calculate(obj, &param);
            break;
        case chassis_rotate_run:
            Chassis_calculate(obj, &param);
            break;
        default:
            break;
    }
    // Chassis_Motor_Update(obj);
}