#include "can_motor.h"

#include "bsp_can.h"
#include "stdlib.h"
#include "string.h"

can_motor* instances[2][3][4];  // instances用于存放can_motor*类型的指向电机实体的指针，在每个电机初始化时被填充
uint8_t motors_id[2][3][8];  // motors_id 被填充为1表示此处有电机被注册，为0表示此处没有电机被注册
const uint32_t identifiers[3] = {0x200, 0x1FF, 0x2FF};

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t* data,
                         uint32_t len);

void Can_Motor_FeedbackData_Update(can_motor* obj, uint8_t* data);

void Can_Motor_Driver_Init() {
    memset(&instances, 0, sizeof(instances));
    memset(&motors_id, 0, sizeof(motors_id));
    BSP_CAN_RegisterRxCallback(0, CanMotor_RxCallBack);
    BSP_CAN_RegisterRxCallback(1, CanMotor_RxCallBack);
}

can_motor* Can_Motor_Create(can_motor_config* config) {
    can_motor* obj = (can_motor*)malloc(sizeof(can_motor));
    motors_id[obj->config.bsp_can_index][obj->config.motor_model]
             [obj->config.motor_set_id] = 1;

    switch (config->motor_model) {
        case MODEL_2006:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][0][config->motor_set_id] = obj;
            } else {
                instances[config->bsp_can_index][1][config->motor_set_id - 4] =obj;
            }
            break;
        case MODEL_3508:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][0][config->motor_set_id] = obj;
            } else {
                instances[config->bsp_can_index][1][config->motor_set_id - 4] =obj;
            }
            break;
        case MODEL_6020:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][1][config->motor_set_id] = obj;
            } else {
                instances[config->bsp_can_index][2][config->motor_set_id - 4] =obj;
            }
            break;
        default:
            break;
    }
    return obj;
    // cvector_pushback(motor_instances, &obj);
}

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t* data,uint32_t len) {
    uint32_t model_3508_2006_id = identifier - 0x200;
    uint32_t model_6020_id = identifier - 0x204;
    if (motors_id[can_id][MODEL_2006][model_3508_2006_id]) {
        Can_Motor_FeedbackData_Update(instances[can_id][model_3508_2006_id > 4 ? 1 : 0][model_3508_2006_id > 4 ? model_3508_2006_id - 4: model_3508_2006_id],data);
    } else if (motors_id[can_id][MODEL_3508][model_3508_2006_id]) {
        Can_Motor_FeedbackData_Update(instances[can_id][model_3508_2006_id > 4 ? 1 : 0][model_3508_2006_id > 4 ? model_3508_2006_id - 4: model_3508_2006_id],data);
    } else if (motors_id[can_id][MODEL_6020][model_6020_id]) {
        Can_Motor_FeedbackData_Update(instances[can_id][model_6020_id > 4 ? 1 : 0][model_6020_id > 4 ? model_6020_id - 4 : model_6020_id],data);
    }
}



void Can_Motor_FeedbackData_Update(can_motor* obj, uint8_t* data) {
    obj->fdbPosition = data[0] << 8 | data[1];
    obj->fdbSpeed = data[2] << 8 | data[3];
    obj->electric_current = data[4] << 8 | data[5];
    if (obj->config.motor_model == MODEL_2006) {
        obj->temperature = 0;
    } else {
        obj->temperature = data[6];
    }
}

void Can_Motor_Calc() {}