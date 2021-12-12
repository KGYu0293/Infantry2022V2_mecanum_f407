#include "bsp_uart.h"

#include "cvector.h"
#include "usart.h"

#define DEVICE_UART_CNT 2

typedef struct BSP_UART_Typedef_t
{
    UART_HandleTypeDef *port; //自定义uart编号
    cvector *call_backs;
} BSP_UART_Typedef;

BSP_UART_Typedef uart_ports[DEVICE_UART_CNT];

void BSP_UART_Init()
{
    uart_ports[0].port = &huart3;
    uart_ports[0].call_backs = cvector_create(sizeof(uart_rx_callback));

    uart_ports[1].port = &huart6;
    uart_ports[1].call_backs = cvector_create(sizeof(uart_rx_callback));
}

// 注册回调函数
void BSP_UART_RegisterRxCallback(uint8_t uart_index, uart_rx_callback func)
{
    cvector_pushback(uart_ports[uart_index].call_backs, &func);
}

//三种模式的发送函数
void BSP_UART_Send_blocking(uint8_t uart_index, uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(uart_ports[uart_index].port, data, len, 20);
}
void BSP_UART_Send_IT(uint8_t uart_index, uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit_IT(uart_ports[uart_index].port, data, len);
}
void BSP_UART_Send_DMA(uint8_t uart_index, uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit_DMA(uart_ports[uart_index].port, data, len);
}

/**
 * @brief 串口空闲中断（中断回调）函数
 * @param 串口号
 * @retval None
 * @note  放在"stm32f4xx_it.c"里形如"void
 * USART2_IRQHandler(void)"类的函数中，只要用了DMA接收的串口都放
 * 具体位置：系统调用的HAL_UART_IRQHandler函数下面，"USER CODE BEGIN USART1_IRQn
 * 1"和"USER CODE END USART1_IRQn 1"两行注释之间
 */
// void BSP_UART_IDLE_RxCallback(UART_HandleTypeDef* huart) {
//     if (huart == uart_ports[0].port) {
//         if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
//             __HAL_UART_CLEAR_IDLEFLAG(huart);
//             HAL_UART_DMAStop(&huart);
//             uint8_t data_length = RX_LEN - __HAL_DMA_GET_COUNTER(huart.hdmarx);
//             PC_DataHandler(data_length);
//             HAL_UART_Receive_DMA(&huart, rx_data, RX_LEN);
//         }
//     }
// }
