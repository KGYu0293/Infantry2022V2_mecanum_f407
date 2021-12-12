#ifndef _BSP_UART_H
#define _BSP_UART_H

#define UART_REMOTE_PORT 0
#define UART_REFEREE_PORT 1

enum BSP_UART_SendMode_e {
    blocking,
    IT,
    DMA
};  //发送模式，堵塞/中断/DMA，其中堵占式不推荐

typedef void (*uart_rx_callback)(uint8_t uart_index, uint8_t* data,
                                 uint32_t len);

void BSP_UART_Init();
void BSP_UART_Send(uint8_t uart_index, uint8_t* data, uint16_t len);
void BSP_UART_RegisterRxCallback(uint8_t uart_index, uart_rx_callback func);
#endif