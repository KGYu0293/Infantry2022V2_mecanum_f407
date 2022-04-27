#ifndef _SUPER_CAP_WULI_H
#define _SUPER_CAP_WULI_H

#include "monitor.h"
#include "stdint.h"

#define SUPER_CAP_WULI_RX_ID 0x211  //雾列超级电容控制板
#define SUPER_CAP_WULI_TX_ID 0x210

typedef struct super_cap_wuli_config_t {
    uint8_t bsp_can_index;
    uint16_t super_cap_wuli_tx_id;
    uint16_t super_cap_wuli_rx_id;
    lost_callback lost_callback;
} super_cap_wuli_config;

typedef struct super_cap_wuli_t {
    super_cap_wuli_config config;
    float voltage_input_fdb;  // 返回值 电容组输入电压
    float voltage_cap_fdb;    // 返回值 电容输出电压
    float current_input_fdb;  // 返回值 电容输入电流
    float power_set_fdb;      // 返回的用户设定值 应与设定值相同
    float power_set;          // 用户设定值

    uint8_t if_supercap_on;  // 电容是否开启 //雾列电容无此功能
    float power_current;     // 电容充电电流
    float cap_percent;       // 电容电量剩余百分比，该电容由电压值测得大致百分比
    monitor_item* monitor;
} Super_cap_wuli;

void Super_cap_wuli_Driver_Init();
Super_cap_wuli* Super_cap_wuli_Create(super_cap_wuli_config* config);
void Super_cap_wuli_Send();
void Super_cap_wuli_FeedbackData_Update(Super_cap_wuli* obj, uint8_t* data);
void Super_cap_wuli_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t* data, uint32_t len);
#endif