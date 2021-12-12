#include "can_motor.h"
#include "string.h"
#include "bsp_can.h"
#include "stdlib.h"

can_motor* instances[2][3][4];
uint8_t motors_id[2][3][8];
const uint32_t identifiers[3] = {0x200, 0x1FF, 0x2FF};

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t* data,
                         uint32_t len);

void Can_Motor_Driver_Init() {
    memset(&instances,0,sizeof(instances));
    memset(&motors_id,0,sizeof(motors_id));
    BSP_CAN_RegisterRxCallback(0, CanMotor_RxCallBack);
    BSP_CAN_RegisterRxCallback(1, CanMotor_RxCallBack);
}

can_motor* Can_Motor_Create(can_motor_config* config) {
    can_motor* obj = (can_motor*)malloc(sizeof(can_motor));
    motors_id[obj->config.bsp_can_index][obj->config.motor_model][obj->config.motor_set_id] = 1;
    switch (config->motor_model) {
        case MODEL_2006:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][0][config->motor_set_id];
            } else {
                instances[config->bsp_can_index][1][config->motor_set_id - 4];
            }
            break;
        case MODEL_3508:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][0][config->motor_set_id];
            } else {
                instances[config->bsp_can_index][1][config->motor_set_id - 4];
            }
            break;
        case MODEL_6020:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][1][config->motor_set_id];
            } else {
                instances[config->bsp_can_index][2][config->motor_set_id - 4];
            }
            break;
        default:
            break;
    }
    return obj;
    // cvector_pushback(motor_instances, &obj);
}

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t* data,
                         uint32_t len) {

    uint32_t model_3508_2006_id = identifier - 0x200;
    uint32_t model_6020_id = identifier - 0x204;
    if(motors_id[can_id][MODEL_2006][model_3508_2006_id]){
        
    }
    else if(motors_id[can_id][MODEL_3508][model_3508_2006_id]){

    }
    else if(motors_id[can_id][MODEL_6020][model_6020_id]){
        
    }
}

void Can_Motor_Calc() {}