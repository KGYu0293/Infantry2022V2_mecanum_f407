#include "super_cap_wuli.h"

#include "bsp_can.h"
#include "bsp_log.h"
#include "cvector.h"
#include "stdlib.h"
#include "string.h"

cvector *super_cap_wuli_instances;  //用动态数组，参照裁判系统

void Super_cap_wuli_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len);

void Super_cap_wuli_FeedbackData_Update(Super_cap_wuli *obj, uint8_t *data);
void Super_cap_wuli_Send();

void Super_cap_wuli_Driver_Init() {
    super_cap_wuli_instances = cvector_create(sizeof(Super_cap_wuli *));
    BSP_CAN_RegisterRxCallback(0, Super_cap_wuli_RxCallBack);
    BSP_CAN_RegisterRxCallback(1, Super_cap_wuli_RxCallBack);
}

Super_cap_wuli *Super_cap_wuli_Create(super_cap_wuli_config *config) {
    Super_cap_wuli *obj = (Super_cap_wuli *)malloc(sizeof(Super_cap_wuli));
    memset(obj, 0, sizeof(Super_cap_wuli));
    obj->config = *config;

    cvector_pushback(super_cap_wuli_instances, &obj);
    BSP_CAN_AddFilter(obj->config.bsp_can_index, obj->config.super_cap_wuli_rx_id);

    obj->monitor = Monitor_Register(obj->config.lost_callback, 5, obj);
    return obj;
}

void Super_cap_wuli_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len) {
    for (size_t i = 0; i < super_cap_wuli_instances->cv_len; i++) {
        Super_cap_wuli *now = *(Super_cap_wuli **)cvector_val_at(super_cap_wuli_instances, i);
        if (can_id == now->config.bsp_can_index && now->config.super_cap_wuli_rx_id == identifier) {
            now->monitor->reset(now->monitor);
            Super_cap_wuli_FeedbackData_Update(now, data);
            Super_cap_wuli_Getpercent(now);
        }
    }
}

void Super_cap_wuli_Getpercent(Super_cap_wuli *obj) {
    //最好用函数拟合
    //或者多描几个点省算力（bushi
    if (obj->voltage_cap_fdb < 15)
        obj->cap_percent = 20;
    else if (obj->voltage_cap_fdb >= 15 && obj->voltage_cap_fdb < 17)
        obj->cap_percent = 35;
    else if (obj->voltage_cap_fdb >= 17 && obj->voltage_cap_fdb < 19)
        obj->cap_percent = 50;
    else if (obj->voltage_cap_fdb >= 19 && obj->voltage_cap_fdb < 21)
        obj->cap_percent = 65;
    else if (obj->voltage_cap_fdb >= 21 && obj->voltage_cap_fdb < 23)
        obj->cap_percent = 80;
    else if (obj->voltage_cap_fdb >= 23 && obj->voltage_cap_fdb < 24.5)
        obj->cap_percent = 95;
    else if (obj->voltage_cap_fdb >= 24.5 && obj->voltage_cap_fdb < 25)
        obj->cap_percent = 100;
}

void Super_cap_wuli_FeedbackData_Update(Super_cap_wuli *obj, uint8_t *data) {
    uint16_t super_cap_wuli_raw[4];
    memcpy(super_cap_wuli_raw, data, sizeof(super_cap_wuli_raw));
    obj->voltage_input_fdb = (float)super_cap_wuli_raw[0] / 100.f;
    obj->voltage_cap_fdb = (float)super_cap_wuli_raw[1] / 100.f;
    obj->current_input_fdb = (float)super_cap_wuli_raw[2] / 100.f;
    obj->power_set_fdb = (float)super_cap_wuli_raw[3] / 100.f;
}

// 调用时，修改结构体中power_set的值
void Super_cap_wuli_Send() {
    static uint8_t data[8] = {0};
    for (size_t i = 0; i < super_cap_wuli_instances->cv_len; i++) {
        Super_cap_wuli *obj = *(Super_cap_wuli **)cvector_val_at(super_cap_wuli_instances, i);
        if (obj == NULL)
            return;
        else {
            uint16_t power_temp = obj->power_set * 100;
            data[0] = (uint8_t)(power_temp >> 8);
            data[1] = (uint8_t)(power_temp);
        }
        BSP_CAN_Send(obj->config.bsp_can_index, obj->config.super_cap_wuli_tx_id, data, 8);
    }
}