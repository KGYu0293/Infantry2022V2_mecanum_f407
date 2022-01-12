/* 基于can_send/can_recv 创建的标准化的电机控制机制*/
#ifndef _CAN_MOTOR_H
#define _CAN_MOTOR_H
#include "circular_queue.h"
#include "pid.h"
#include "stdint.h"
#include "monitor.h"
enum Motor_Model_e { MODEL_3508 = 0, MODEL_2006, MODEL_6020 };

enum Motor_PID_Model_e { CURRENT_LOOP = 0, SPEED_LOOP, POSITION_LOOP };  //速度环/位置环/电流环
enum Motor_FDB_Model_e { MOTOR_FDB = 0, OTHER_FDB };

typedef struct can_motor_config_t {
    uint8_t bsp_can_index;
    uint8_t motor_set_id;  //电调上通过闪灯次数确定的id
    enum Motor_Model_e motor_model;
    enum Motor_PID_Model_e motor_pid_model;
    enum Motor_FDB_Model_e position_fdb_model;
    enum Motor_FDB_Model_e speed_fdb_model;
    struct PID_config_t config_speed;
    struct PID_config_t config_position;
    float* speed_pid_fdb;
    float* position_pid_fdb;
    lost_callback lost_callback;
} can_motor_config;

typedef struct can_motor_t {
    can_motor_config config;
    enum  {MOTOR_ENABLE, MOTOR_STOP} enable;
    short fdbPosition;       //电机的编码器反馈值
    short last_fdbPosition;  //电机上次的编码器反馈值
    short fdbSpeed;          //电机反馈的转速/rpm
    short electric_current;  //电机实际转矩电流
    short round;             //电机转过的圈数
    uint8_t temperature;     //电机温度
    float real_position;        //过零处理后的电机转子位置
    float last_real_position;   //上次真实转过的角度
    struct PID_t speed_pid;     //速度环pid
    struct PID_t position_pid;  //位置环pid
    short current_output;

    float line_speed;                //线速度（m/s，根据角速度算出）
    circular_queue* position_queue;  //计算角速度的循环队列
    float position_sum;                       //队列中所有值的和
    float velocity;                  //用电机编码器计算出来的角速度（单位：度每秒）
    monitor_item* monitor;
} can_motor;

void Can_Motor_Driver_Init();
can_motor* Can_Motor_Create(can_motor_config* config);
void Can_Motor_Calc_Send();
#endif