#include "VL53L0x.h"

#include "bsp_def.h"
#include "bsp_delay.h"
#include "bsp_gpio.h"
#include "bsp_log.h"
#include "cvector.h"
#include "stdlib.h"
#include "string.h"
#include "i2c.h"

uint8_t VL53L0x_SendData[2] = {0x01};
uint8_t VL53L0x_RecData[5];

//寄存器操作函数
#define VL53L0x_add 0x52  // VL53L0x的IIC器件地址

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID 0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14

cvector *VL53L0x_instances;

void VL53L0x_Driver_Init() {
    VL53L0x_instances = cvector_create(sizeof(VL53L0x *));
}

VL53L0x *VL53L0x_Create(VL53L0x_config *config) {
    VL53L0x *obj = (VL53L0x *)malloc(sizeof(VL53L0x));
    memset(obj, 0, sizeof(VL53L0x));
    obj->config = *config;

    cvector_pushback(VL53L0x_instances, &obj);

    obj->monitor = Monitor_Register(obj->config.lost_callback, 20, obj);

    obj->soft_i2c = Soft_I2c_Create(&config->soft_i2c_config);

    Soft_I2c_WriteData(obj->soft_i2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, *VL53L0x_SendData);
    // HAL_I2C_Mem_Write(&hi2c2, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 100);
    
    bsp_delay_us(500);
    VL53L0x_SendData[1] = 100;
    while (VL53L0x_SendData[1]--) {
        bsp_delay_us(1);
        Soft_I2c_ReadData(obj->soft_i2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, (VL53L0x_RecData + 4), 1);
        // HAL_I2C_Mem_Read(&hi2c2,VL53L0x_add+1,VL53L0X_REG_RESULT_RANGE_STATUS,I2C_MEMADD_SIZE_8BIT,(VL53L0x_RecData + 4),1,100);
        if (VL53L0x_RecData[4] & 0x01)
            break;
    }

    return obj;
}

void VL53L0x_Enable(VL53L0x *obj) {
    BSP_GPIO_Set(obj->config.bsp_gpio_xshut_index, 1);
}

void VL53L0x_Disable(VL53L0x *obj) {
    BSP_GPIO_Set(obj->config.bsp_gpio_xshut_index, 0);
}

// VL53L0x转换一次
void VL53L0x_StartConversion(VL53L0x *obj) {
    uint8_t VL53L0x_SendData[1] = {0x01};
    if (!Soft_I2c_WriteData(obj->soft_i2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, *VL53L0x_SendData))
    // if (!HAL_I2C_Mem_Write(&hi2c2, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 100))
        obj->monitor->reset(obj->monitor);
}

uint16_t makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

// VL53L0x读取距离等数据反馈信息
void VL53L0x_ReadDistance(VL53L0x *obj) {
    //记录历史有效值
    obj->dist_last = obj->distValid;
    obj->last_acnt = obj->acnt;

    Soft_I2c_ReadData(obj->soft_i2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, obj->vtemp, 12);
    // HAL_I2C_Mem_Read(&hi2c2,VL53L0x_add+1,VL53L0X_REG_RESULT_RANGE_STATUS,I2C_MEMADD_SIZE_8BIT, obj->vtemp ,12,100);

    obj->acnt = makeuint16(obj->vtemp[7], obj->vtemp[6]);
    obj->scnt = makeuint16(obj->vtemp[9], obj->vtemp[8]);
    obj->dist = makeuint16(obj->vtemp[11], obj->vtemp[10]);
    obj->DeviceRangeStatusInternal = ((obj->vtemp[0] & 0x78) >> 3);

    //提取有效值
    if (obj->dist <= 0x0014)  //距离数据无效
        obj->distValid = obj->dist_last;
    else  //有效
        obj->distValid = obj->dist;
}
