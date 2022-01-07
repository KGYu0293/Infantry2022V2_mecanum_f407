#include "can_recv.h"

#include "bsp_can.h"
#include "crc16.h"
#include "cvector.h"
#include "stdio.h"

cvector* can_recv_instances;
void CanRecv_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t* data,
                        uint32_t len);

void CanRecv_Driver_Init() {
    can_recv_instances = cvector_create(sizeof(can_recv*));
    BSP_CAN_RegisterRxCallback(0, CanRecv_RxCallBack);
    BSP_CAN_RegisterRxCallback(1, CanRecv_RxCallBack);
}

can_recv* CanRecv_Create(can_recv_config* config) {
    can_recv* obj = (can_recv*)malloc(sizeof(can_recv));
    obj->config = *config;
    obj->data_rx.len = obj->config.data_len;
    obj->data_rx.data = (uint8_t*)malloc(obj->config.data_len);
    obj->buf_len = obj->config.data_len + 5;
    obj->rxbuf = (uint8_t*)malloc(obj->buf_len);
    obj->recv_len = 0;
    obj->recv_status = 0;
    obj->data_updated = 0;
    cvector_pushback(can_recv_instances, &obj);
    BSP_CAN_AddFilter(obj->config.bsp_can_index, obj->config.can_identifier);
    obj->monitor = Monitor_Register(obj->config.lost_callback, 20, obj);
    return obj;
}

void CanRecv_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t* data,
                        uint32_t len) {
    for (size_t i = 0; i < can_recv_instances->cv_len; ++i) {
        can_recv* now = *(can_recv**)cvector_val_at(can_recv_instances, i);
        if (now->config.bsp_can_index == can_id &&
            now->config.can_identifier == identifier) {
            if (data[0] == 's') {
                now->recv_status = 1;
            }
            if (now->recv_status) {
                if (now->recv_len + len > now->buf_len) {
                    now->recv_status = 0;
                    now->recv_len = 0;
                    return;
                }
                memcpy(now->rxbuf + now->recv_len, data, len);
                now->recv_len += len;
                if (now->rxbuf[now->recv_len - 1] == 'e' &&
                    now->recv_len == now->buf_len) {
                    if (CheckVaild(now->rxbuf + 1, now->buf_len - 2)) {
                        BufferToData(now->rxbuf + 1, &now->data_rx);
                        now->recv_status = 0;
                        now->recv_len = 0;
                        now->data_updated = 1;
                        if (now->config.notify_func != NULL) {
                            now->config.notify_func(now);
                        }
                    }
                }
            }
            return;
        }
    }
}