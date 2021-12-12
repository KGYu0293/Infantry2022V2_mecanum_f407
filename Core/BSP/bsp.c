#include "bsp.h"

void BSP_Layer_Init(){
    BSP_GPIO_Init();
    BSP_SPI_Init();
    BSP_PWM_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_Log_Init();
}