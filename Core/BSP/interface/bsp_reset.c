#include "bsp_reset.h"
#include "stm32f4xx_hal.h"

void BSP_soft_reset(void)
{
	__set_PRIMASK(1);
	HAL_NVIC_SystemReset();
}