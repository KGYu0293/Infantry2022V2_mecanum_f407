/**
 * @file           : adrc.c
 * @brief          : ADRC算法实现
 * @Author         : 李鸣航
 * @Date           : 2022-04-09 14:48
 * @LastEditTime   : 2022-05-04 11:04
 * @Note           : 使用该算法之前请务必阅读说明文档，禁止随意调参
 * @Copyright(c)   : 哈尔滨工业大学（深圳）南工骁鹰机器人队版权所有 Critical HIT copyrighted
 */

#include "adrc.h"

#include <string.h>

#include "math.h"

/**
 * @brief      :浮点数限幅函数
 * @attention  :无
 * @param {float} amt 要比较的值
 * @param {float} low 下限
 * @param {float} high 上限
 */
float ConstrainFloat(float amt, float low, float high) {
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/**
 * @brief      :符号函数，x大于0时返回1，x小于0时返回-1，x等于0时返回0
 * @attention  :无
 * @return  {int} 1  x大于0
 *          {int} -1 x小于0
 *          {int} 0  x等于0
 * @param {float} x 需要判断的变量
 */
int sgn(float x) {
    if (x > 1E-6)
        return 1;
    else if (x < 1E-6)
        return -1;
    else
        return 0;
}

/**
 * @brief      :二维符号函数
 * @attention  :无
 * @param {float} x 输入1
 * @param {float} d 输入2
 */
int fsg(float x, float d) {
    return ((sgn(x + d) - sgn(x - d)) / 2);
}

/**
 * @brief      :fal是一种工业上运用很广的非线性结构函数，其可用于观测器或者函数滤波器，本质上是对工业上“大误差，小增益；小误差，大增益”经验知识的一个数学拟合
 * @attention  :
 *  调参说明：
 *  1.a为非线性因子，合理选择可以极大地改变控制效果，对于比例误差（即正常相减得到的误差）和积分误差，我们需要在小误差时选用大增益，在大误差时选用小增益，此时0<a<1
 *                                               对于微分误差，我们需要在小误差时选择小增益，在大误差时选择大增益，此时a>1
 *      同时，对于a来说，a越小，跟踪效果越好，但滤波效果会变差，反之同理
 *  2.delta为滤波因子，一般取5T≤delta≤10T，其中T为采样周期，在代码中体现为操作系统中任务的运行周期
 *      同时，对于delta来说，delta越大，滤波效果越好，但跟踪效果会越延迟
 * @return  {float} 经过滤波后的误差输出信号
 * @param {float} e 带有噪声的误差输入信号
 * @param {float} a 非线性因子
 * @param {float} delta 滤波因子
 */
float fal(float e, float a, float delta) {
    if (fabs(e) <= delta) {
        return (e / pow(delta, 1 - a));
    } else {
        return (sgn(e) * pow(fabs(e), a));
    }
}

/**
 * @brief      :fst是一种最速跟踪函数，可以快速有效的跟踪信号，其物理意义是在限制了加速度r的情况下，如何最快达到预期值
 * @attention  :
 *  调参说明：
 *  1. 需要配合跟踪微分器一起使用，按照跟踪微分器的传参和迭代方式进行
 *  2. r为速度因子，决定跟踪速度，r越大，快速性越好，但大的r容易引起超调和振荡
 *  3. h为滤波因子，h越大，静态误差越小，刚开始带来的“超调”越小，初始的误差越小；但会导致上升过慢，快速性不好，一般取步长的2倍
 * @return  {float} 输出跟踪的微分信号
 * @param {float} x1_delta 误差输入信号
 * @param {float} x2 误差的微分信号
 * @param {float} r 速度因子
 * @param {float} h0 滤波因子
 */
float fst(float x1_delta, float x2, float r, float h0) {
    float d, y, a0, a, a1, a2, a3;
    d = r * h0 * h0;
    a0 = h0 * x2;
    y = x1_delta + a0;
    a1 = sqrt(d * (d + 8 * fabs(y)));
    a2 = a0 + sgn(y) * (a1 - d) / 2;
    a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
    return (-r * (a / d) * fsg(a, d) - r * sgn(a) * (1 - fsg(a, d)));
}

/**
 * @brief      : 跟踪微分器
 * @attention  : 无
 * @return  {*}
 * @param {ADRC_t*} adrc_data adrc数据结构体
 */
void TDFunction(ADRC_t* adrc_data) {
    float last_v1, last_v2;
    last_v1 = adrc_data->prog.v1;
    last_v2 = adrc_data->prog.v2;
    adrc_data->prog.v1 = last_v1 + adrc_data->adrc_config.td.h * last_v2;
    adrc_data->prog.v2 = last_v2 + adrc_data->adrc_config.td.h *
                                       fst(last_v1 - adrc_data->prog.ref, last_v2,
                                           adrc_data->adrc_config.td.r, adrc_data->adrc_config.td.h0);
}
/**
 * @brief      : 非线性反馈
 * @attention  : 无
 * @return  {*}
 * @param {ADRC_t*} adrc_data adrc数据结构体
 */
void NLSEFFunction(ADRC_t* adrc_data) {
    adrc_data->prog.u0 = adrc_data->adrc_config.nlsef.Kp *
                             fal(adrc_data->prog.e1, adrc_data->adrc_config.nlsef.alpha1, adrc_data->adrc_config.nlsef.delta) +
                         adrc_data->adrc_config.nlsef.Kd *
                             fal(adrc_data->prog.e2, adrc_data->adrc_config.nlsef.alpha2, adrc_data->adrc_config.nlsef.delta);
}

/**
 * @brief      : 扩张观测器
 * @attention  : 无
 * @return  {*}
 * @param {ADRC_t*} adrc_data adrc数据结构体
 */
void ESOFunction(ADRC_t* adrc_data) {
    float epsilon, last_z1, last_z2, last_z3;
    last_z1 = adrc_data->prog.z1;
    last_z2 = adrc_data->prog.z2;
    last_z3 = adrc_data->prog.z3;
    epsilon = last_z1 - adrc_data->prog.fdb;
    adrc_data->prog.z1 = last_z1 + adrc_data->adrc_config.td.h *
                                       (last_z2 - adrc_data->adrc_config.nlsef.Kp * epsilon);
    adrc_data->prog.z2 = last_z2 + adrc_data->adrc_config.td.h *
                                       (last_z3 - adrc_data->adrc_config.nlsef.Kd * fal(epsilon, adrc_data->adrc_config.nlsef.alpha1, adrc_data->adrc_config.nlsef.delta) +
                                        adrc_data->adrc_config.eso.b * adrc_data->prog.output);
    adrc_data->prog.z3 = last_z3 - adrc_data->adrc_config.td.h *
                                       adrc_data->adrc_config.eso.beta3 * fal(epsilon, adrc_data->adrc_config.nlsef.alpha2, adrc_data->adrc_config.nlsef.delta);
}

/**
 * @brief      : ADRC总函数
 * @attention  : 无
 * @return  {*}
 * @param {ADRC_t*} adrc_data adrc数据结构体
 */
void ADRCFunction(ADRC_t* adrc_data) {
    TDFunction(adrc_data);
    adrc_data->prog.e1 = adrc_data->prog.v1 - adrc_data->prog.z1;
    adrc_data->prog.e2 = adrc_data->prog.v2 - adrc_data->prog.z2;
    NLSEFFunction(adrc_data);
    adrc_data->prog.output = adrc_data->prog.u0 - adrc_data->prog.z3 / adrc_data->adrc_config.eso.b;
    ESOFunction(adrc_data);
}

/**
 * @brief      : ADRC结构体配置初始化
 * @attention  : 调参说明在adrc.h中，切勿自行更改参数
 * @return  {*}
 */
void ADRC_SetConfig(ADRC_Config_t* adrc_config,
                    float r, float h, float h0,
                    float Kp, float Kd, float alpha1, float alpha2, float delta,
                    float beta1, float beta2, float beta3, float b) {
    adrc_config->td.r = r;
    adrc_config->td.h = h;
    adrc_config->td.h0 = h0;
    adrc_config->nlsef.Kp = Kp;
    adrc_config->nlsef.Kd = Kd;
    adrc_config->nlsef.alpha1 = alpha1;
    adrc_config->nlsef.alpha2 = alpha2;
    adrc_config->nlsef.delta = delta;
    adrc_config->eso.beta1 = beta1;
    adrc_config->eso.beta2 = beta2;
    adrc_config->eso.beta3 = beta3;
    adrc_config->eso.b = b;
}

/**
 * @brief      :ADRC结构体初始化
 * @attention  :无
 * @param {ADRC_t*} adrc_data
 * @param {ADRC_Config_t*} adrc_config
 */
void ADRC_Init(ADRC_t* adrc_data, ADRC_Config_t* adrc_config) {
    adrc_data->adrc_config = *adrc_config;
    memset(&adrc_data->prog, 0, sizeof(adrc_data->prog));
}