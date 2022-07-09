#include "soft_i2c.h"

#include <stdlib.h>
#include <string.h>

#include "bsp_delay.h"
#include "bsp_gpio.h"

#define IIC_SDA_H(obj) BSP_GPIO_Set(obj->config.bsp_gpio_sda_index, 1)
#define IIC_SDA_L(obj) BSP_GPIO_Set(obj->config.bsp_gpio_sda_index, 0)
#define IIC_SCL_H(obj) BSP_GPIO_Set(obj->config.bsp_gpio_scl_index, 1)
#define IIC_SCL_L(obj) BSP_GPIO_Set(obj->config.bsp_gpio_scl_index, 0)
#define IIC_SDA_Read(obj, data) BSP_GPIO_Read(obj->config.bsp_gpio_sda_index, &data)

void IIC_SDA_Out(Soft_I2c* obj) {
    BSP_GPIO_Reinit(obj->config.bsp_gpio_sda_index,1);
}

void IIC_SDA_In(Soft_I2c* obj) {
    BSP_GPIO_Reinit(obj->config.bsp_gpio_sda_index,0);
}

void IIC_Start(Soft_I2c* obj) {
    IIC_SDA_Out(obj);
    IIC_SDA_H(obj);
    IIC_SCL_H(obj);
    bsp_delay_us(5);
    IIC_SDA_L(obj);
    bsp_delay_us(5);
    IIC_SCL_L(obj);
}

void IIC_Stop(Soft_I2c* obj) {
    IIC_SDA_Out(obj);
    IIC_SCL_L(obj);
    IIC_SDA_L(obj);
    bsp_delay_us(5);
    IIC_SCL_H(obj);
    bsp_delay_us(5);
    IIC_SDA_H(obj);
}

void IIC_Ack(Soft_I2c* obj, uint8_t re) {
    IIC_SDA_Out(obj);
    IIC_SCL_L(obj);
    bsp_delay_us(5);
    if (re)
        IIC_SDA_H(obj);
    else
        IIC_SDA_L(obj);
    bsp_delay_us(5);
    IIC_SCL_H(obj);
    bsp_delay_us(5);
    IIC_SCL_L(obj);
    bsp_delay_us(5);
}

uint8_t IIC_WaitAck(Soft_I2c* obj) {
    uint16_t Out_Time = 0;
    IIC_SDA_H(obj);
    IIC_SDA_In(obj);
    bsp_delay_us(5);
    IIC_SCL_H(obj);
    bsp_delay_us(5);

    uint8_t data;
    IIC_SDA_Read(obj, data);
    while (data) {
        Out_Time++;
        if (Out_Time>100) {
            IIC_Stop(obj);
            return 0xff;
        }
        IIC_SDA_Read(obj, data);
    }
    IIC_SCL_L(obj);
    return 0;
}

void IIC_WriteBit(Soft_I2c* obj, uint8_t Temp) {
    uint8_t i;
    IIC_SDA_Out(obj);
    IIC_SCL_L(obj);
    for (i = 0; i < 8; i++) {
        if (Temp & 0x80)
            IIC_SDA_H(obj);
        else
            IIC_SDA_L(obj);
        Temp <<= 1;
        bsp_delay_us(5);
        IIC_SCL_H(obj);
        bsp_delay_us(5);
        IIC_SCL_L(obj);
        bsp_delay_us(5);
    }
}

uint8_t IIC_ReadBit(Soft_I2c* obj) {
    uint8_t i, Temp = 0;
    IIC_SDA_In(obj);
    uint8_t data;
    for (i = 0; i < 8; i++) {
        IIC_SCL_L(obj);
        bsp_delay_us(5);
        IIC_SCL_H(obj);
        Temp <<= 1;
        IIC_SDA_Read(obj, data);
        if (data)
            Temp++;
        bsp_delay_us(5);
    }
    IIC_SCL_L(obj);
    return Temp;
}

//写入一个字节
uint8_t Soft_I2c_WriteData(Soft_I2c* obj,uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    IIC_Start(obj);
    IIC_WriteBit(obj,dev_addr);
    if (IIC_WaitAck(obj) == 0xff)
        return 0xff;
    IIC_WriteBit(obj,reg_addr);
    if (IIC_WaitAck(obj) == 0xff)
        return 0xff;
    IIC_WriteBit(obj,data);

    if (IIC_WaitAck(obj) == 0xff)
        return 0xff;
    IIC_Stop(obj);
    return 0;
}

uint8_t Soft_I2c_ReadData(Soft_I2c* obj,uint8_t dev_addr, uint8_t reg_addr, uint8_t* pdata, uint8_t count) {
    uint8_t i;

    IIC_Start(obj);
    IIC_WriteBit(obj,dev_addr);
    if (IIC_WaitAck(obj) == 0xff)
        return 0xff;
    IIC_WriteBit(obj,reg_addr);
    if (IIC_WaitAck(obj) == 0xff)
        return 0xff;
    IIC_Start(obj);
    IIC_WriteBit(obj,dev_addr + 1);
    if (IIC_WaitAck(obj) == 0xff)
        return 0xff;
    for (i = 0; i < (count - 1); i++) {
        *pdata = IIC_ReadBit(obj);
        IIC_Ack(obj,0);
        pdata++;
    }
    *pdata = IIC_ReadBit(obj);
    IIC_Ack(obj,1);
    IIC_Stop(obj);
    return 0;
}

Soft_I2c* Soft_I2c_Create(Soft_I2c_config* config) {
    Soft_I2c* obj = (Soft_I2c*)malloc(sizeof(Soft_I2c));
    memset(obj, 0, sizeof(Soft_I2c));
    obj->config = *config;
    return obj;
}
