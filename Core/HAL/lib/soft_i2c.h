#ifndef _SOFT_I2C_H_
#define _SOFT_I2C_H_

#include "stdint.h"

#pragma pack(1)
typedef struct Soft_I2c_config_t {
    uint8_t bsp_gpio_sda_index;
    uint8_t bsp_gpio_scl_index;
} Soft_I2c_config;
typedef struct Soft_I2c_t {
    Soft_I2c_config config;
    uint8_t data;
} Soft_I2c;
#pragma pack()

Soft_I2c* Soft_I2c_Create(Soft_I2c_config* config);
uint8_t Soft_I2c_ReadData(Soft_I2c* obj,uint8_t dev_addr, uint8_t reg_addr, uint8_t * pdata, uint8_t count);
uint8_t Soft_I2c_WriteData(Soft_I2c* obj,uint8_t dev_addr,uint8_t reg_addr,uint8_t data);

#endif