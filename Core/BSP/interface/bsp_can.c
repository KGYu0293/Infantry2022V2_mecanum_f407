/**
  ******************************************************************************
  * 文件名          : bsp_can.c
  * 创建时间        : 2021.12.07
  * 作者            : 陈迅  
  ******************************************************************************
  * 1.本代码包含大量中文注释，请以UTF-8编码格式打开
  * 2.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
  * 3.Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  * 说明：
  * 
  * 
  ******************************************************************************
  */
#include "bsp_can.h"

#include "can.h"
#include "cvector.h"

#define DEVICE_CAN_CNT 2    //主控共使用（拥有）的can总线数量
#define ID_MAX 0x07FF
#define ID_NOTSET 0x800     //未使用过滤器时的填充值

typedef struct BSP_CanTypeDef_t {
    CAN_HandleTypeDef *device;  //自定义总线编号
    uint32_t tx_mailbox;
    cvector *call_backs;
    uint32_t fifo;
    uint32_t bank_prefix;       //不同can对应的过滤器相关参数值
    uint16_t filters[FILTER_MAX_CNT];  //按标准帧ID定义
} BSP_CanTypeDef;

BSP_CanTypeDef can_devices[DEVICE_CAN_CNT]; //定义对应数量的can外设，即代码中定义can总线

void BSP_CAN_Init() {
    can_devices[0].device = &hcan1;
    can_devices[0].fifo = CAN_RX_FIFO0;
    // can_devices[0].tx_mailbox = (uint32_t *)CAN_TX_MAILBOX0;
    can_devices[0].bank_prefix = 0;
    can_devices[0].call_backs = cvector_create(sizeof(can_rx_callback));    //创建用于储存回调函数的向量组，存储方式为其头指针

    can_devices[1].device = &hcan2;
    can_devices[1].fifo = CAN_RX_FIFO1;
    // can_devices[1].tx_mailbox = (uint32_t *)CAN_TX_MAILBOX1;
    can_devices[1].bank_prefix = 14;
    can_devices[1].call_backs = cvector_create(sizeof(can_rx_callback));

    for (size_t d = 0; d < DEVICE_CAN_CNT; ++d) {
        for (size_t i = 0; i < FILTER_MAX_CNT; ++i) {
            can_devices[d].filters[i] = ID_NOTSET;
        }
    }

    HAL_CAN_Start(can_devices[0].device);   //将HAL库的对应功能函数封装，实现映射和隔离
    HAL_CAN_Start(can_devices[1].device);
    HAL_CAN_ActivateNotification(can_devices[0].device, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(can_devices[1].device, CAN_IT_RX_FIFO1_MSG_PENDING);

}

//  仅对HAL库的can发送函数进行了形式上的封装，实际参数依然依赖外部传参
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t *data,
                  uint32_t len) {
    CAN_TxHeaderTypeDef txconf;
    txconf.StdId = identifier;
    txconf.IDE = CAN_ID_STD;
    txconf.RTR = CAN_RTR_DATA;
    txconf.DLC = len;
    HAL_CAN_AddTxMessage(can_devices[can_id].device, &txconf, data,
                         &can_devices[can_id].tx_mailbox);
}


void update_filter(uint8_t can_id, uint32_t filter_index) {
    CAN_FilterTypeDef tmp;
    tmp.FilterMode = CAN_FILTERMODE_IDLIST;
    tmp.FilterScale = CAN_FILTERSCALE_16BIT;
    tmp.FilterFIFOAssignment = can_devices[can_id].fifo;
    tmp.SlaveStartFilterBank = 14;
    uint32_t group_index = filter_index / 4;
    uint16_t *filter_data = can_devices[can_id].filters;
    tmp.FilterIdLow = filter_data[group_index * 4] << 5;
    tmp.FilterIdHigh = filter_data[group_index * 4 + 1] << 5;
    tmp.FilterMaskIdLow = filter_data[group_index * 4 + 2] << 5;
    tmp.FilterMaskIdHigh = filter_data[group_index * 4 + 3] << 5;
    tmp.FilterBank = group_index + can_devices[can_id].bank_prefix;
    uint8_t id_all_notset = 1;
    for (size_t i = 0; i < 4; ++i) {
        if (filter_data[group_index * 4 + i] != ID_NOTSET) id_all_notset = 0;
    }
    if (id_all_notset)
        tmp.FilterActivation = CAN_FILTER_DISABLE;
    else
        tmp.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(can_devices[can_id].device, &tmp);
}

void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter) {
    // uint32_t filter_index = can_devices[can_id].filter_cnt++;
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT;
         ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == ID_NOTSET) {
            can_devices[can_id].filters[filter_index] = filter;
            update_filter(can_id, filter_index);
            return;
        }
    }
}

void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter) {
    // uint32_t filter_index = can_devices[can_id].filter_cnt++;
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT;
         ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == filter) {
            can_devices[can_id].filters[filter_index] = ID_NOTSET;
            update_filter(can_id, filter_index);
        }
    }
}

void BSP_CAN_RegisterRxCallback(uint8_t can_id, can_rx_callback func) {
    cvector_pushback(can_devices[can_id].call_backs, &func);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == can_devices[0].device) {
        uint8_t bsp_can_rxbuf0[8];
        CAN_RxHeaderTypeDef rxconf;
        HAL_CAN_GetRxMessage(hcan, can_devices[0].fifo, &rxconf,
                             bsp_can_rxbuf0);
        for (size_t i = 0; i < can_devices[0].call_backs->cv_len; ++i) {
            can_rx_callback funcnow = *(can_rx_callback *)cvector_val_at(
                can_devices[0].call_backs, i);
            funcnow(0, rxconf.StdId, bsp_can_rxbuf0, rxconf.DLC);
        }
    }
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == can_devices[1].device) {
        uint8_t bsp_can_rxbuf1[8];
        CAN_RxHeaderTypeDef rxconf;
        HAL_CAN_GetRxMessage(hcan, can_devices[1].fifo, &rxconf,
                             bsp_can_rxbuf1);
        for (size_t i = 0; i < can_devices[1].call_backs->cv_len; ++i) {
            can_rx_callback funcnow = *(can_rx_callback *)cvector_val_at(
                can_devices[1].call_backs, i);
            funcnow(1, rxconf.StdId, bsp_can_rxbuf1, rxconf.DLC);
        }
    }
}