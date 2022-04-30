#ifndef _BSP_UART_H
#define _BSP_UART_H
#include "stdint.h"
#include "usart.h"

#define UART_REMOTE_PORT 0
#define UART_REFEREE_PORT 1

typedef void (*uart_rx_callback)(uint8_t uart_index, uint8_t *data, uint32_t len);

void BSP_UART_Init();
void BSP_UART_RegisterRxCallback(uint8_t uart_index, uart_rx_callback func);

void BSP_UART_Send_blocking(uint8_t uart_index, uint8_t *data, uint16_t len);
void BSP_UART_Send_IT(uint8_t uart_index, uint8_t *data, uint16_t len);
void BSP_UART_Send_DMA(uint8_t uart_index, uint8_t *data, uint16_t len);
// 以队列方式发送，推荐使用，请严格注意，使用该种方式就不要使用其它方式发送数据
void BSP_UART_Send_queue(uint8_t uart_index, uint8_t *data, uint16_t len);
// DMA接收处理
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);
#endif