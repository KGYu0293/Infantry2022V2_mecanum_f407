#ifndef __VL53L0X_H
#define __VL53L0X_H	

#include "monitor.h"
#include "stdint.h"
#include "soft_i2c.h"

#pragma pack(1)
typedef struct VL53L0x_config_t {
    Soft_I2c_config soft_i2c_config;
    uint8_t bsp_gpio_xshut_index;
    lost_callback lost_callback;
} VL53L0x_config;

typedef struct VL53L0x_t {
    VL53L0x_config config;

    Soft_I2c* soft_i2c;

    uint8_t vtemp[12];
	uint16_t acnt;		//环境统计,激光光强
	uint16_t last_acnt;		//上一次环境统计,激光光强
	uint16_t scnt;		//信号数
	uint16_t dist;		//距离，单位mm	最原始数据
	uint8_t DeviceRangeStatusInternal;
	
	uint16_t dist_last;		//历史有效值，用来判断
	
	uint16_t distValid;		//原始距离值提取后的有效值
	uint16_t dist_buff[15];		//测距的滑窗缓存
	uint16_t distValidFinal;
	
	float GroundDis;
	float GroundDis_last;
	
	float Speed;
	float Speed_last;
	
	uint16_t distFilted;	//卡尔曼滤波后的距离值
	uint16_t distFilted_1;//卡尔曼后 + 一阶低通滤波
	
	uint8_t Flag_OverRange;	//超出量程标志位

    monitor_item* monitor;
} VL53L0x;
#pragma pack()

void VL53L0x_Driver_Init();
VL53L0x* VL53L0x_Create(VL53L0x_config* config);
void VL53L0x_Enable(VL53L0x *obj);
void VL53L0x_Disable(VL53L0x *obj);
void VL53L0x_StartConversion(VL53L0x *obj); //VL53L0x转换一次
void VL53L0x_ReadDistance(VL53L0x *obj); //VL53L0x读取距离等数据反馈信息
#endif