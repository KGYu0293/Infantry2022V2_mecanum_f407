#include "bsp_gpio.h"

#include "bsp_def.h"
#include "gpio.h"

typedef struct BSP_GPIO_Typedef_t {
    GPIO_TypeDef* base;
    uint16_t pin;
    uint8_t mode;
} BSP_GPIO_Typedef;

BSP_GPIO_Typedef gpio_ports[DEVICE_GPIO_CNT];

void BSP_GPIO_Init() {
    gpio_ports[0].base = GPIO_0_BASE;
    gpio_ports[0].pin = GPIO_0_PIN;
    gpio_ports[0].mode = GPIO_0_MODE;

    gpio_ports[1].base = GPIO_1_BASE;
    gpio_ports[1].pin = GPIO_1_PIN;
    gpio_ports[1].mode = GPIO_1_MODE;
}

void BSP_GPIO_Set(uint8_t gpio_index, uint8_t status) {
    if (gpio_ports[gpio_index].mode == GPIO_OUTPUT_MODE) {
        HAL_GPIO_WritePin(gpio_ports[gpio_index].base, gpio_ports[gpio_index].pin, status);
    }
}

void BSP_GPIO_Read(uint8_t gpio_index, uint8_t* data) {
    if (gpio_ports[gpio_index].mode == GPIO_INPUT_MODE) {
        (*data) = HAL_GPIO_ReadPin(gpio_ports[gpio_index].base, gpio_ports[gpio_index].pin);
    }
}