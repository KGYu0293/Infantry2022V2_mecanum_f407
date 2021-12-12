#include "bsp_uart.h"

#include "usart.h"

#define DEVICE_UART_CNT 2

typedef struct BSP_UART_Typedef_t {
    UART_HandleTypeDef *port;
} BSP_UART_Typedef;

BSP_UART_Typedef uart_ports[DEVICE_UART_CNT];

void BSP_UART_Init()
{
    uart_ports[0].port = &huart3;
    uart_ports[1].port = &huart6;
}
