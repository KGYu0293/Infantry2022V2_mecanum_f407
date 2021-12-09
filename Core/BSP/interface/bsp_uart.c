#include "bsp_uart.h"

#include "usart.h"

#define BSP_UART_CNT 2

typedef struct BSP_UART_Typedef_t {
    UART_HandleTypeDef *port;
} BSP_UART_Typedef;

