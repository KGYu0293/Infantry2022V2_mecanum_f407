#include "bsp_uart.h"

#include "cvector.h"
#include "usart.h"

#define DEVICE_UART_CNT 2

typedef struct BSP_UART_Typedef_t {
    UART_HandleTypeDef* port;  //自定义uart编号
    cvector* call_backs;
    enum {
        blocking,
        IT,
        DMA
    } send_mode;  //发送模式，堵塞/中断/DMA，其中堵占式不推荐
} BSP_UART_Typedef;

BSP_UART_Typedef uart_ports[DEVICE_UART_CNT];

void BSP_UART_Init() {
    uart_ports[0].port = &huart3;
    uart_ports[0].send_mode = blocking;
    uart_ports[0].call_backs = cvector_create(sizeof(uart_rx_callback));

    uart_ports[1].port = &huart6;
    uart_ports[1].send_mode = DMA;
    uart_ports[1].call_backs = cvector_create(sizeof(uart_rx_callback));
}

void BSP_UART_Send(uint8_t uart_index, uint8_t* data, uint32_t len) {
    switch (uart_ports[uart_index].send_mode) {
        case blocking:
            HAL_UART_Transmit(uart_ports[uart_index].port, data, len, 10);
            break;
        case IT:
            HAL_UART_Transmit_IT(uart_ports[uart_index].port, data, len);
            break;
        case DMA:
            HAL_UART_Transmit_DMA(uart_ports[uart_index].port, data, len);
            break;
        default:
            break;
    }
}
