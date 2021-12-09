#include "bsp_uart.h"

#include "usart.h"

#define BSP_UART_CNT 2

typedef struct BSP_SPI_Typedef_t {
    SPI_HandleTypeDef *port;
} BSP_SPI_Typedef;