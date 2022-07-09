#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H
#include "stdint.h"

void BSP_GPIO_Init();
void BSP_GPIO_Reinit(uint8_t gpio_index, uint8_t mode);
void BSP_GPIO_Read(uint8_t gpio_index, uint8_t* data);
void BSP_GPIO_Set(uint8_t gpio_index, uint8_t status);
void BSP_GPIO_Toggle(uint8_t gpio_index);

#endif