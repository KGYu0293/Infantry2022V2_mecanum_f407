/**
  ******************************************************************************
  * 文件名          : indicator_led.c
  * 创建时间        : 2022.5.21
  * 作者            : 余泽恺
  * ----------------------------------------------------------------------------
  * 最近修改时间    : 
  * 修改人          : 
  ******************************************************************************
  * 1.本代码包含大量中文注释，请以UTF-8编码格式打开
  * 2.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
  * 3.Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  * 说明：使用开发板上的LED
  * 令LED按时间不断闪烁，以此监测开发板是否正常运行
  * 
  ******************************************************************************
  */
#include "indicator_led.h"

#include "bsp_gpio.h"
#include "cvector.h"

cvector *indicator_led_instances;

void indicator_led_Driver_Init(void) { indicator_led_instances = cvector_create(sizeof(Indicator_led *)); }

Indicator_led *indicator_led_Create(Indicator_led_config *config) {
    Indicator_led *obj = (Indicator_led *)malloc(sizeof(Indicator_led));
    memset(obj, 0, sizeof(Indicator_led));
    obj->config = *config;
    cvector_pushback(indicator_led_instances, &obj);
    return obj;
}

void indicator_led_Update(Indicator_led *obj) {
    switch (obj->led_cnt) {
        case 0:
            BSP_GPIO_Toggle(obj->config.bsp_gpio_led1_index);
            break;
        case 1:
            BSP_GPIO_Toggle(obj->config.bsp_gpio_led2_index);
            break;
        case 2:
            BSP_GPIO_Toggle(obj->config.bsp_gpio_led3_index);
            break;
        default:
            break;
    }
    if (obj->led_cnt >= 2)
        obj->led_cnt = 0;
    else
        obj->led_cnt++;
}

void indicator_led_Update_ALL(void) {
    for (size_t i = 0; i < indicator_led_instances->cv_len; i++) {
        Indicator_led *indicator_led_now = *((Indicator_led **)cvector_val_at(indicator_led_instances, i));
        indicator_led_Update(indicator_led_now);
    }
}