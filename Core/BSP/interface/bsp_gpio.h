#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H
#include "stdint.h"

//此处可以预定义一些接口ID
#define GPIO_BMI088_ACCEL_NS 0
#define GPIO_BMI088_GYRO_NS 1

void BSP_GPIO_Init();
void BSP_GPIO_Read(uint8_t gpio_index,uint8_t* data);
void BSP_GPIO_Set(uint8_t gpio_index,uint8_t status);

#endif