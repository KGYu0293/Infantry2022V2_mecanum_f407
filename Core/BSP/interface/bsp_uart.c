#include "bsp_uart.h"

#include "cvector.h"
#include "usart.h"

#define DEVICE_UART_CNT 2
#define BSP_UART_DMA_BUFF_SIZE 255 //DMA缓冲区大小，一次传输数据不应超过该长度

typedef struct BSP_UART_Typedef_t
{
    UART_HandleTypeDef *port; //自定义uart编号
    cvector *call_backs;

    uint8_t rx_buff[BSP_UART_DMA_BUFF_SIZE]; //DMA接收数组，为了存放DMA搬运的串口数据
} BSP_UART_Typedef;

BSP_UART_Typedef uart_ports[DEVICE_UART_CNT];

void BSP_UART_Init()
{
    uart_ports[0].port = &huart3;
    uart_ports[0].call_backs = cvector_create(sizeof(uart_rx_callback));
    __HAL_UART_ENABLE_IT(uart_ports[0].port, UART_IT_IDLE); //使能串口空闲中断
    HAL_UART_Receive_DMA(uart_ports[0].port, uart_ports->rx_buff, BSP_UART_DMA_BUFF_SIZE);

    uart_ports[1].port = &huart6;
    uart_ports[1].call_backs = cvector_create(sizeof(uart_rx_callback));
    __HAL_UART_ENABLE_IT(uart_ports[1].port, UART_IT_IDLE); //使能串口空闲中断
    HAL_UART_Receive_DMA(uart_ports[1].port, uart_ports->rx_buff, BSP_UART_DMA_BUFF_SIZE);
}

// 注册回调函数
void BSP_UART_RegisterRxCallback(uint8_t uart_index, uart_rx_callback func)
{
    cvector_pushback(uart_ports[uart_index].call_backs, &func);
}

//三种模式的发送函数(仅简单封装)
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
 * @note  放在"stm32f4xx_it.c"里形如"void USART2_IRQHandler(void)"类的函数中，只要用了DMA接收的串口都放
 * 具体位置：系统调用的HAL_UART_IRQHandler函数下面，"USER CODE BEGIN USART1_IRQn 1"和"USER CODE END USART1_IRQn 1"两行注释之间
 */
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart == uart_ports[0].port)
    {
        //由于已经在上面的if判断完成，下面用传入的huart代替uart_ports[x].port即可，两者必定是同一个
        if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart); //清除空闲中断标志（否则会一直不断进入中断）
            // 下面进行空闲中断相关处理
            // USAR_UART_IDLECallback(huart); //调用中断处理函数
            HAL_UART_DMAStop(huart);    //暂时停止本次DMA传输，进行数据处理
            uint8_t data_length = BSP_UART_DMA_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx); //计算接收到的数据长度
            for (size_t i = 0; i < uart_ports[0].call_backs->cv_len; i++)   //调用回调函数对数据进行处理
            {
                uart_rx_callback funcnow = *(uart_rx_callback *)cvector_val_at(uart_ports[0].call_backs, i);
                funcnow(0,uart_ports[0].rx_buff,data_length);
            }
            HAL_UART_Receive_DMA(huart, uart_ports[0].rx_buff, BSP_UART_DMA_BUFF_SIZE); //重启开始DMA传输 每次255字节数据
        }
    }
    if (huart == uart_ports[1].port)
    {
        if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            HAL_UART_DMAStop(huart);
            uint8_t data_length = BSP_UART_DMA_BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
            for (size_t i = 0; i < uart_ports[1].call_backs->cv_len; i++)
            {
                uart_rx_callback funcnow = *(uart_rx_callback *)cvector_val_at(uart_ports[0].call_backs, i);
                funcnow(1,uart_ports[1].rx_buff,data_length);
            }
            HAL_UART_Receive_DMA(huart, uart_ports[1].rx_buff, BSP_UART_DMA_BUFF_SIZE);
        }
    }
}
