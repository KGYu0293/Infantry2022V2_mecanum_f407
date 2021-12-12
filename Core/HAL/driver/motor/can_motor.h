/* 基于can_send/can_recv 创建的标准化的电机控制机制*/
#ifndef _CAN_MOTOR_H
#define _CAN_MOTOR_H
#include "can_recv.h"
#include "can_send.h"
#include "datatypes.h"
#include "pid.h"
#include "stdint.h"

enum Motor_Model_e { MODEL_3508 = 0, MODEL_2006, MODEL_6020};


typedef struct can_motor_config_t {
    uint8_t bsp_can_index;
	uint8_t motor_set_id;  //电调上通过闪灯次数确定的id
	enum Motor_Model_e motor_model;
    struct PID_config_t config_speed;
    struct PID_config_t config_position;
} can_motor_config;

typedef struct can_motor_t {
    can_motor_config config;

    short fdbPosition;       //电机的编码器反馈值
    short last_fdbPosition;  //电机上次的编码器反馈值
    short fdbSpeed;          //电机反馈的转速/rpm
    short electric_current;  //电机实际转矩电流
    short round;             //电机转过的圈数
    uint8_t temperature;     //电机温度

    int32_t real_position;       //过零处理后的电机转子位置
    int32_t last_real_position;  //上次真实转过的角度
    struct PID_t speed_pid;      //速度环pid
    struct PID_t position_pid;   //位置环pid

    float line_speed;      //线速度（m/s，根据角速度算出）
    int position_buf[24];  //计算角速度的数组
    int index;             //数组编号标志
    int velocity;  //用电机编码器计算出来的角速度（单位：度每秒）

} can_motor;

void Can_Motor_Driver_Init();
can_motor *Can_Motor_Create(can_motor_config *config);
void Can_Motor_Calc();
#endif