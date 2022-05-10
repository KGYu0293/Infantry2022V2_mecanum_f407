#include "pwm_servo.h"

#include "cvector.h"

#define CCR_START 500
#define CCR_END 2500

cvector *servo_instances;

void Servo_Update(Servo *obj);
uint32_t servo_ccr_calc(Servo *obj);

void Servo_Driver_Init(void) { servo_instances = cvector_create(sizeof(Servo *)); }

Servo *Servo_Create(Servo_config *config) {
    Servo *obj = (Servo *)malloc(sizeof(Servo));
    memset(obj, 0, sizeof(Servo));
    obj->config = *config;
    cvector_pushback(servo_instances, &obj);
    BSP_PWM_Start(obj->config.bsp_pwm_index);
    obj->pos_servo_control = obj->config.initial_angle;
    return obj;
}

/**
 * @brief 舵机角度解算
        以舵机为参考系设置角度，根据实际需要测出所需要的舵机参考系下的角度
 * @param 绝对角度 obj->set_angle
 * @retval None
 */
uint32_t servo_ccr_calc(Servo *obj) {
    uint32_t ccr = 0;
    switch (obj->config.model) {
        case MODEL_SPEED:
            if (obj->speed_servo_control.speed > 100) obj->speed_servo_control.speed = 100;
            switch (obj->speed_servo_control.direc) {
                case servo_hold:
                    ccr = 1500;
                    break;
                case servo_forward:
                    ccr = 1500 + 10 * obj->speed_servo_control.speed;
                    break;
                case servo_reverse:
                    ccr = 1500 - 10 * obj->speed_servo_control.speed;
                    break;
                default:
                    break;
            }
            break;
        case MODEL_POS:
            if (obj->pos_servo_control > obj->config.max_angle) obj->pos_servo_control = obj->config.max_angle;
            ccr = CCR_START + ((CCR_END - CCR_START) * obj->pos_servo_control / obj->config.max_angle);
            break;
        default:
            break;
    }

    return ccr;
}

void Servo_Update(Servo *obj) {
    // 即时控制
    BSP_PWM_SetCCR(obj->config.bsp_pwm_index, servo_ccr_calc(obj));
    // 限速控制
    // 为满足需要 可以在这里添加降速到达目标位置的代码 示例如下
    // uint32_t ccr_now = obj->ccr = BSP_PWM_ReadCCR(obj->config.bsp_pwm_index);
    // uint32_t crc_target = servo_ccr_calc(obj);
    // if (crc_target > ccr_now)
    //     BSP_PWM_SetCCR(obj->config.bsp_pwm_index, ccr_now + x);
    // else
    //     BSP_PWM_SetCCR(obj->config.bsp_pwm_index, ccr_now - x);
}

void Servo_Update_ALL(void) {
    for (size_t i = 0; i < servo_instances->cv_len; ++i) {
        Servo *servo_now = *((Servo **)cvector_val_at(servo_instances, i));
        Servo_Update(servo_now);
    }
}