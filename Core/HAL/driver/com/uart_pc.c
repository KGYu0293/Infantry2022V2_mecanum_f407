#include <stdlib.h>
#include <string.h>
#include <uart_pc.h>
uartpc* UartPC_Create(uartpc_config* config) {
    uartpc* obj = (uartpc*)malloc(sizeof(uartpc));
    memset(obj, 0, sizeof(uartpc));
    obj->config = *config;
    uart_recv_config recv_config;
    uart_send_config send_config;
    recv_config.bsp_uart_index = config->bsp_uart_index;
    recv_config.uart_identifier = config->recv_identifer;
    recv_config.data_len = sizeof(pc_recv);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = config->lost_callback;
    //
    send_config.bsp_uart_index = config->bsp_uart_index;
    send_config.data_len = sizeof(pc_send);
    send_config.uart_identifier = config->send_identifer;
    obj->recv = UartRecv_Create(&recv_config);
    obj->send = UartSend_Create(&send_config);
    obj->pc_recv_data = (pc_recv*)obj->recv->data_rx.data;
    obj->data_updated = &obj->recv->data_updated;
    return obj;
}

void UartPC_Send(uartpc* obj, pc_send* data) {
    UartSend_Send(obj->send, (uint8_t*)data);
}