/**
 ******************************************************************************
 * 文件名        : remote.c
 * 文件描述      ：PID控制算法
 * 创建时间      ：2019.11.9
 * 作者          ：刘文熠
 *-----------------------------------------------------------------------------
 * 最近修改时间  ：2021.5.8
 * 修改人        ：刘思宇
 * 内容          ：加入PI分离模式
 ******************************************************************************
 * 1.本代码基于STMF427IIT6开发，编译环境为Keil 5，基于FreeRTOS进行开发。
 * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
 * 3.本代码包含大量中文注释，请以UTF-8编码格式打开
 * 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
 *
 * Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
 ******************************************************************************
 */

#include "pid.h"

#include "math.h"

/**
 * @brief 符号函数
 * @param input 输入
 * @retval 输入的符号，正负1或0
 */
float_t fsgn(float input) {
    return (input != 0.0f ? (input < 0.0f ? -1.0f : 1.0f) : 0.0f);
}

/**
 * @brief PID计算函数，位置式和增量式合在一起
 * @param PID结构体
 * @retval None
 */

void PID_Init(struct PID_t* pid, struct PID_config_t* config) {
    pid->config = *config;
}

void PID_Calc(struct PID_t* pid) {
    pid->error[2] = pid->error[1];        //上上次误差
    pid->error[1] = pid->error[0];        //上次误差
    pid->error[0] = pid->ref - pid->fdb;  //本次误差

    if (pid->config.PID_Mode == PID_POSITION)  //位置式PID
    {
        pid->error_sum += pid->error[0];  //积分上限判断
        if (pid->error_sum > pid->config.error_max)
            pid->error_sum = pid->config.error_max;
        if (pid->error_sum < -pid->config.error_max)
            pid->error_sum = -pid->config.error_max;

        pid->output = pid->config.KP * pid->error[0] +
                      pid->config.KI * pid->error_sum +
                      pid->config.KD * (pid->error[0] - pid->error[1]);
    }

    else if (pid->config.PID_Mode == PID_DELTA)  //增量式PID
    {
        pid->output += pid->config.KP * (pid->error[0] - pid->error[1]) +
                       pid->config.KI * (pid->error[0] - 2.0f * pid->error[1] +
                                         pid->error[2]) +
                       pid->config.KI * pid->error[0];
    }

    else if (pid->config.PID_Mode ==
             PID_COMP_POSITION)  // PI分离模式 用于摩擦轮和拨弹
    {
        pid->error_delta = pid->error[0] - pid->error[1];
        if (pid->error[0] > pid->config.range_rough)  // bangbang
        {
            pid->output = pid->config.outputMax;
            pid->error_sum = 0;
        } else if (pid->error[0] < -pid->config.range_rough)  // bangbang
        {
            pid->output = -pid->config.outputMax;
            pid->error_sum = 0;
        } else if (fabsf(pid->error[0]) < pid->config.range_fine)  //细调
        {
            pid->error_sum += pid->error[0];  //积分上限判断
            if (pid->error_sum > pid->config.error_max)
                pid->error_sum = pid->config.error_max;
            if (pid->error_sum < -pid->config.error_max)
                pid->error_sum = -pid->config.error_max;
            pid->output = pid->config.KP_fine * pid->error[0] +
                          pid->config.KI * pid->error_sum +
                          pid->config.KD * pid->error_delta;
        } else  //粗调
        {
            pid->output = pid->config.KP *
                              (pid->error[0] +
                               fsgn(pid->error[0]) * pid->config.compensation) +
                          pid->config.KD * pid->error_delta;
            pid->error_sum = fsgn(pid->error[0]) * pid->config.error_preload;
        }
    }

    /* 输出上限 */
    if (pid->output > pid->config.outputMax)
        pid->output = pid->config.outputMax;
    if (pid->output < -pid->config.outputMax)
        pid->output = -pid->config.outputMax;
}

void PID_SetConfig(struct PID_config_t* obj, float kp, float ki, float kd,
                   float errormax, float outputmax) {
    obj->PID_Mode = PID_POSITION;
    obj->KP = kp;
    obj->KI = ki;
    obj->KD = kd;
    obj->error_max = errormax;
    obj->outputMax = outputmax;
}