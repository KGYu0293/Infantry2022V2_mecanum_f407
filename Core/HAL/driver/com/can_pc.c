#include "can_pc.h"

#include <stdlib.h>
#include <string.h>
canpc* CanPC_Create(canpc_config* config) {
    canpc* obj = (canpc*)malloc(sizeof(canpc));
    memset(obj, 0, sizeof(canpc));
    obj->config = *config;
    can_recv_config recv_config;
    can_send_config send_config;
    recv_config.bsp_can_index = config->bsp_can_index;
    recv_config.can_identifier = config->recv_identifer;
    recv_config.data_len = sizeof(pc_recv);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = config->lost_callback;

    send_config.bsp_can_index = config->bsp_can_index;
    send_config.can_identifier = config->send_identifer;
    send_config.data_len = sizeof(pc_send);
    obj->recv = CanRecv_Create(&recv_config);
    obj->send = CanSend_Create(&send_config);
    obj->pc_recv_data = (pc_recv*)obj->recv->data_rx.data;
    obj->data_updated = &obj->recv->data_updated;
    return obj;
}

void CanPC_Send(canpc* obj, pc_send* data) {
    CanSend_Send(obj->send, (uint8_t*)data);
}