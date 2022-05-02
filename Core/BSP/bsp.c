#include "bsp.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_log.h"
#include "stdint.h"

void BSP_Layer_Init(){
    BSP_GPIO_Init();
    BSP_SPI_Init();
    BSP_PWM_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_Log_Init();
}