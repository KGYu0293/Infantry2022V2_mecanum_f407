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

    gpio_ports[2].base = GPIO_2_BASE;
    gpio_ports[2].pin = GPIO_2_PIN;
    gpio_ports[2].mode = GPIO_2_MODE;

    gpio_ports[3].base = GPIO_3_BASE;
    gpio_ports[3].pin = GPIO_3_PIN;
    gpio_ports[3].mode = GPIO_3_MODE;

    gpio_ports[4].base = GPIO_4_BASE;
    gpio_ports[4].pin = GPIO_4_PIN;
    gpio_ports[4].mode = GPIO_4_MODE;

    gpio_ports[5].base = GPIO_5_BASE;
    gpio_ports[5].pin = GPIO_5_PIN;
    gpio_ports[5].mode = GPIO_5_MODE;
}

void BSP_GPIO_Set(uint8_t gpio_index, uint8_t status) {
    if (gpio_ports[gpio_index].mode == GPIO_OUTPUT_MODE) {
        HAL_GPIO_WritePin(gpio_ports[gpio_index].base, gpio_ports[gpio_index].pin, status);
    }
}

void BSP_GPIO_Toggle(uint8_t gpio_index) {
    if (gpio_ports[gpio_index].mode == GPIO_OUTPUT_MODE) {
        HAL_GPIO_TogglePin(gpio_ports[gpio_index].base, gpio_ports[gpio_index].pin);
    }
}

void BSP_GPIO_Read(uint8_t gpio_index, uint8_t* data) {
    if (gpio_ports[gpio_index].mode == GPIO_INPUT_MODE) {
        (*data) = HAL_GPIO_ReadPin(gpio_ports[gpio_index].base, gpio_ports[gpio_index].pin);
    }
}