#include "can_motor.h"

#include "bsp_can.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "string.h"

#define VELOCITY_WINDOW 5
#define FDB_INIT_VALUE (-10000)

can_motor *motor_instances[2][3][4];  // motor_instances用于存放can_motor*类型的指向电机实体的指针，在每个电机初始化时被填充
uint8_t motors_id[2][3][8];           // motors_id
                                      // 被填充为1表示此处有电机被注册，为0表示此处没有电机被注册
const uint32_t identifiers[3] = {0x200, 0x1FF, 0x2FF};

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len);

void Can_Motor_FeedbackData_Update(can_motor *obj, uint8_t *data);
void Can_Motor_Send(uint8_t can_id, uint16_t identifier, short motor_data_id1, short motor_data_id2, short motor_data_id3, short motor_data_id4);

void Can_Motor_Driver_Init() {
    memset(&motor_instances, 0, sizeof(motor_instances));
    memset(&motors_id, 0, sizeof(motors_id));
    BSP_CAN_RegisterRxCallback(0, CanMotor_RxCallBack);
    BSP_CAN_RegisterRxCallback(1, CanMotor_RxCallBack);
}

can_motor *Can_Motor_Create(can_motor_config *config) {
    can_motor *obj = (can_motor *)malloc(sizeof(can_motor));
    memset(obj, 0, sizeof(can_motor));
    obj->position_queue = create_circular_queue(sizeof(float), VELOCITY_WINDOW);
    obj->config = *config;
    obj->position_sum = 0;
    obj->fdbPosition = FDB_INIT_VALUE;
    motors_id[obj->config.bsp_can_index][obj->config.motor_model][obj->config.motor_set_id - 1] = 1;
    switch (config->motor_model) {
        case MODEL_2006:
            if (config->motor_set_id < 5) {
                motor_instances[config->bsp_can_index][0][config->motor_set_id - 1] = obj;
            } else {
                motor_instances[config->bsp_can_index][1][config->motor_set_id - 5] = obj;
            }
            BSP_CAN_AddFilter(obj->config.bsp_can_index, 0x200 + config->motor_set_id);
            break;
        case MODEL_3508:
            if (config->motor_set_id < 5) {
                motor_instances[config->bsp_can_index][0][config->motor_set_id - 1] = obj;
            } else {
                motor_instances[config->bsp_can_index][1][config->motor_set_id - 5] = obj;
            }
            BSP_CAN_AddFilter(obj->config.bsp_can_index, 0x200 + config->motor_set_id);
            break;
        case MODEL_6020:
            if (config->motor_set_id < 5) {
                motor_instances[config->bsp_can_index][1][config->motor_set_id - 1] = obj;
            } else {
                motor_instances[config->bsp_can_index][2][config->motor_set_id - 5] = obj;
            }
            BSP_CAN_AddFilter(obj->config.bsp_can_index, 0x204 + config->motor_set_id);
            break;
        default:
            break;
    }
    if (obj->config.speed_fdb == NULL) {
        obj->config.speed_fdb = &obj->velocity;
    }
    if (obj->config.position_fdb == NULL) {
        obj->config.position_fdb = &obj->real_position;
    }
    obj->motor_controller = create_controller(&obj->config.motor_controller_config);
    obj->monitor = Monitor_Register(obj->config.lost_callback, 5, obj);

    obj->enable = MOTOR_STOP;
    return obj;
}

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len) {
    uint32_t model_3508_2006_id = identifier - 0x200;
    uint32_t model_6020_id = identifier - 0x204;
    // if (model_3508_2006_id > 8) return;
    if (model_3508_2006_id < 9 && motors_id[can_id][MODEL_2006][model_3508_2006_id - 1]) {
        Can_Motor_FeedbackData_Update(motor_instances[can_id][model_3508_2006_id < 5 ? 0 : 1][model_3508_2006_id < 5 ? model_3508_2006_id - 1 : model_3508_2006_id - 5], data);
    } else if (model_3508_2006_id < 9 && motors_id[can_id][MODEL_3508][model_3508_2006_id - 1]) {
        Can_Motor_FeedbackData_Update(motor_instances[can_id][model_3508_2006_id < 5 ? 0 : 1][model_3508_2006_id < 5 ? model_3508_2006_id - 1 : model_3508_2006_id - 5], data);
    } else if (model_6020_id < 9 && motors_id[can_id][MODEL_6020][model_6020_id - 1]) {
        Can_Motor_FeedbackData_Update(motor_instances[can_id][model_6020_id < 5 ? 1 : 2][model_6020_id < 5 ? model_6020_id - 1 : model_6020_id - 5], data);
    }
}

void Can_Motor_FeedbackData_Update(can_motor *obj, uint8_t *data) {
    obj->last_fdbPosition = obj->fdbPosition;
    obj->fdbPosition = ((short)data[0]) << 8 | data[1];
    if (obj->last_fdbPosition == FDB_INIT_VALUE) obj->last_fdbPosition = obj->fdbPosition;
    obj->fdbSpeed = ((short)data[2]) << 8 | data[3];
    obj->electric_current = ((short)data[4]) << 8 | data[5];
    if (obj->config.motor_model == MODEL_2006) {
        obj->temperature = 0;
    } else {
        obj->temperature = data[6];
    }
    if (obj->fdbPosition - obj->last_fdbPosition > 4096)
        obj->round--;
    else if (obj->fdbPosition - obj->last_fdbPosition < -4096)
        obj->round++;
    obj->last_real_position_8192 = obj->real_position_8192;
    obj->real_position_8192 = obj->fdbPosition + obj->round * 8192;
    obj->real_position = obj->real_position_8192 * 360.0 / 8192.0;

    float position_delta = obj->real_position_8192 - obj->last_real_position_8192;
    if (obj->position_queue->cq_len == VELOCITY_WINDOW) {
        float *now = circular_queue_pop(obj->position_queue);
        obj->position_sum -= *now;
    }
    circular_queue_push(obj->position_queue, &position_delta);
    obj->position_sum += position_delta;
    float vnow = obj->position_sum * 43.9453125 / obj->position_queue->cq_len;
    obj->velocity = 0.2f * obj->velocity + 0.8f * vnow;
    obj->monitor->reset(obj->monitor);
}

void Can_Motor_Calc_Send() {
    for (int can_index = 0; can_index < 2; can_index++) {
        for (int identifier = 0; identifier < 3; identifier++) {
            uint8_t identifier_send = 0;
            short buf[4] = {0};
            for (int id = 0; id < 4; id++) {
                can_motor *obj = motor_instances[can_index][identifier][id];
                if (obj == NULL) continue;
                //电机掉线判断，如果同一个包内的所有的电机都掉线，那么这个包将不会发出
                if (is_Offline(obj->monitor)) continue;
                identifier_send = 1;
                if (obj->config.position_fdb_model == OTHER_FDB) {
                    obj->motor_controller->fdb_position = *obj->config.position_fdb;
                } else {
                    obj->motor_controller->fdb_position = obj->real_position;
                }
                if (obj->config.speed_fdb_model == OTHER_FDB) {
                    obj->motor_controller->fdb_speed = *obj->config.speed_fdb;
                } else {
                    obj->motor_controller->fdb_speed = obj->fdbSpeed * 6;
                }

                controller_calc(obj->motor_controller);
                buf[id] = (short)obj->motor_controller->output;
                if (obj->config.output_model == MOTOR_OUTPUT_REVERSE) buf[id] *= -1;
                if (obj->enable == MOTOR_STOP) {
                    buf[id] = 0;
                }
            }
            // 如果此标识符(identifier)对应的四个电机里至少有一个被注册，就发送这个标识符的报文，如果全部没有被注册，则这个标识符无需发送
            if (identifier_send) {
                Can_Motor_Send(can_index, identifiers[identifier], buf[0], buf[1], buf[2], buf[3]);
            }
        }
    }
}

void Can_Motor_Send(uint8_t can_id, uint16_t identifier, short motor_data_id1, short motor_data_id2, short motor_data_id3, short motor_data_id4) {
    static uint8_t data[8];
    data[0] = (uint8_t)(motor_data_id1 >> 8);
    data[1] = (uint8_t)(motor_data_id1 & 0x00FF);
    data[2] = (uint8_t)(motor_data_id2 >> 8);
    data[3] = (uint8_t)(motor_data_id2 & 0x00FF);
    data[4] = (uint8_t)(motor_data_id3 >> 8);
    data[5] = (uint8_t)(motor_data_id3 & 0x00FF);
    //对于标识符为0x2FF的情况，则data[6]和data[7]为NULL
    if (identifier == 0x200 || identifier == 0x1FF) {
        data[6] = (uint8_t)((motor_data_id4 & 0xFF00) >> 8);
        data[7] = (uint8_t)(motor_data_id4 & 0x00FF);
    }
    BSP_CAN_Send(can_id, identifier, data, 8);
}
