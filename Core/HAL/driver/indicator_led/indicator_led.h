#ifndef _INDICATOR_LED_H_
#define _INDICATOR_LED_H_

#include "stdint.h"
typedef struct Indicator_led_config_t{
    uint8_t bsp_gpio_led1_index;
    uint8_t bsp_gpio_led2_index;
    uint8_t bsp_gpio_led3_index;
}Indicator_led_config;

typedef struct Indicator_led_t{
    Indicator_led_config config;
    uint8_t led_cnt;
}Indicator_led;

void indicator_led_Driver_Init(void);
Indicator_led* indicator_led_Create(Indicator_led_config* config);
void indicator_led_Update_ALL(void);

#endif
