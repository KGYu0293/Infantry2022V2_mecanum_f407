#ifndef _H_UART_RECV_H
#define _H_UART_RECV_H
#include "datatypes.h"
#include "stdint.h"
struct uart_recv_config_t;
struct uart_recv_t;

typedef struct uart_recv_config_t uart_recv_config;
typedef struct uart_recv_t uart_recv;
typedef void (*uart_recv_notify)(uart_recv* obj);


struct uart_recv_config_t{
    uint8_t bsp_uart_index;
    uint8_t data_len;
    uint16_t uart_identifier;
    uart_recv_notify notify_func;
};

struct uart_recv_t {
    uart_recv_config config;
    general_data data_rx;
    uint8_t buf_len;
    uint8_t recv_len;
    uint8_t recv_status;
    uint8_t data_updated;
    uint8_t* rxbuf;
};

void UartRecv_Driver_Init();
uart_recv* UartRecv_Create(uart_recv_config* config);

#endif