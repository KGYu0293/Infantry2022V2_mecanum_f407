#include "can_motor.h"

#include "bsp_can.h"
#include "stdlib.h"
#include "string.h"

can_motor *instances[2][3][4];  // instances用于存放can_motor*类型的指向电机实体的指针，在每个电机初始化时被填充
uint8_t motors_id[2][3][8];     // motors_id
                                // 被填充为1表示此处有电机被注册，为0表示此处没有电机被注册
const uint32_t identifiers[3] = {0x200, 0x1FF, 0x2FF};

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len);

void Can_Motor_FeedbackData_Update(can_motor *obj, uint8_t *data);
void Can_Motor_Send(uint8_t can_id, uint16_t identifier, uint16_t motor_data_id1, uint16_t motor_data_id2, uint16_t motor_data_id3, uint16_t motor_data_id4);

void Can_Motor_Driver_Init() {
    memset(&instances, 0, sizeof(instances));
    memset(&motors_id, 0, sizeof(motors_id));
    BSP_CAN_RegisterRxCallback(0, CanMotor_RxCallBack);
    BSP_CAN_RegisterRxCallback(1, CanMotor_RxCallBack);
}

can_motor *Can_Motor_Create(can_motor_config *config) {
    can_motor *obj = (can_motor *)malloc(sizeof(can_motor));
    motors_id[obj->config.bsp_can_index][obj->config.motor_model][obj->config.motor_set_id - 1] = 1;

    switch (config->motor_model) {
        case MODEL_2006:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][0][config->motor_set_id - 1] = obj;
            } else {
                instances[config->bsp_can_index][1][config->motor_set_id - 5] = obj;
            }
            break;
        case MODEL_3508:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][0][config->motor_set_id - 1] = obj;
            } else {
                instances[config->bsp_can_index][1][config->motor_set_id - 5] = obj;
            }
            break;
        case MODEL_6020:
            if (config->motor_set_id < 5) {
                instances[config->bsp_can_index][1][config->motor_set_id - 1] = obj;
            } else {
                instances[config->bsp_can_index][2][config->motor_set_id - 5] = obj;
            }
            break;
        default:
            break;
    }
    return obj;
    // cvector_pushback(motor_instances, &obj);
}

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len) {
    uint32_t model_3508_2006_id = identifier - 0x200;
    uint32_t model_6020_id = identifier - 0x204;
    if (motors_id[can_id][MODEL_2006][model_3508_2006_id - 1]) {
        Can_Motor_FeedbackData_Update(instances[can_id][model_3508_2006_id < 5 ? 0 : 1][model_3508_2006_id < 5 ? model_3508_2006_id - 1 : model_3508_2006_id - 5], data);
    } else if (motors_id[can_id][MODEL_3508][model_3508_2006_id - 1]) {
        Can_Motor_FeedbackData_Update(instances[can_id][model_3508_2006_id < 5 ? 0 : 1][model_3508_2006_id < 5 ? model_3508_2006_id - 1 : model_3508_2006_id - 5], data);
    } else if (motors_id[can_id][MODEL_6020][model_6020_id - 1]) {
        Can_Motor_FeedbackData_Update(instances[can_id][model_6020_id < 5 ? 1 : 2][model_6020_id < 5 ? model_6020_id - 1 : model_6020_id - 5], data);
    }
}

void Can_Motor_FeedbackData_Update(can_motor *obj, uint8_t *data) {
    obj->fdbPosition = ((short)data[0]) << 8 | data[1];
    obj->fdbSpeed = ((short)data[2]) << 8 | data[3];
    obj->electric_current = ((short)data[4]) << 8 | data[5];
    if (obj->config.motor_model == MODEL_2006) {
        obj->temperature = 0;
    } else {
        obj->temperature = data[6];
    }
}

void Can_Motor_Calc_Send() {
    for (int index = 0; index < 2; index++) {
        for (int identifier = 0; identifier < 3; identifier++) {
            for (int id = 0; id < 8; id++) {
                if (motors_id[index][identifier][id]) {
                    if (instances[index][identifier][id]->config.motor_pid_model == POSITION_LOOP) {
                        instances[index][identifier][id]->position_pid.fdb = instances[index][identifier][id]->fdbSpeed;
                        PID_Calc(&instances[index][identifier][id]->position_pid);
                        instances[index][identifier][id]->speed_pid.ref = instances[index][identifier][id]->position_pid.output;
                    }
                    //速度环pid解算
                    instances[index][identifier][id]->speed_pid.fdb = instances[index][identifier][id]->fdbSpeed;
                    PID_Calc(&instances[index][identifier][id]->speed_pid);
                }
            }
            //如果此标识符(identifier)对应的四个电机里至少有一个被注册，就发送这个标识符的报文，如果全部没有被注册，则这个标识符无需发送
            if (motors_id[index][identifier][0] | motors_id[index][identifier][1] | motors_id[index][identifier][2] | motors_id[index][identifier][3]) {
                Can_Motor_Send(index, identifiers[identifier],
                               //如果电机被注册，则传递speed_pid.output,如果电机没有被注册，则用0填充无电机的位置
                               (motors_id[index][identifier][0] ? ((uint16_t)instances[index][identifier][0]->speed_pid.output) : 0),
                               (motors_id[index][identifier][1] ? ((uint16_t)instances[index][identifier][1]->speed_pid.output) : 0),
                               (motors_id[index][identifier][2] ? ((uint16_t)instances[index][identifier][2]->speed_pid.output) : 0),
                               (motors_id[index][identifier][3] ? ((uint16_t)instances[index][identifier][3]->speed_pid.output) : 0));
            }
        }
    }
}

void Can_Motor_Send(uint8_t can_id, uint16_t identifier, uint16_t motor_data_id1, uint16_t motor_data_id2, uint16_t motor_data_id3, uint16_t motor_data_id4) {
    uint8_t data[8] = {};
    data[0] = (uint8_t)((motor_data_id1 & 0xFF00) >> 8);
    data[1] = (uint8_t)(motor_data_id1 & 0x00FF);
    data[2] = (uint8_t)((motor_data_id2 & 0xFF00) >> 8);
    data[3] = (uint8_t)(motor_data_id2 & 0x00FF);
    data[4] = (uint8_t)((motor_data_id3 & 0xFF00) >> 8);
    data[5] = (uint8_t)(motor_data_id3 & 0x00FF);
    if (identifier == 0x200 || identifier == 0x1FF)  //对于标识符为0x2FF的情况，则data[6]和data[7]为NULL
    {
        data[6] = (uint8_t)((motor_data_id4 & 0xFF00) >> 8);
        data[7] = (uint8_t)(motor_data_id4 & 0x00FF);
    }
    BSP_CAN_Send(can_id, identifier, data, 8);
}
