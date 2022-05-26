#ifndef _UART_PC_H
#define _UART_PC_H
#include <monitor.h>
#include <pc_data.h>
#include <uart_recv.h>
#include <uart_send.h>

typedef struct uartpc_config_t {
    uint8_t bsp_uart_index;
    uint16_t send_identifer;
    uint16_t recv_identifer;
    lost_callback lost_callback;
} uartpc_config;

typedef struct uartpc_t {
    uartpc_config config;
    uart_send* send;
    uart_recv* recv;
    pc_recv* pc_recv_data;
    uint8_t* data_updated;
} uartpc;

uartpc* UartPC_Create(uartpc_config* config);
void UartPC_Send(uartpc* obj, pc_send* data);

#endif