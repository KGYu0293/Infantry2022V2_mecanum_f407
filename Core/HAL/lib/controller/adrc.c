/**
 * @file           : adrc.c
 * @brief          : ADRC算法实现
 * @Author         : 李鸣航
 * @Date           : 2022-04-09 14:48
 * @LastEditTime   : 2022-04-28 17:03
 * @Note           : 使用该算法之前请务必阅读说明文档，禁止随意调参
 * @Copyright(c)   : 哈尔滨工业大学（深圳）南工骁鹰机器人队版权所有 Critical HIT copyrighted
 */

#include "adrc.h"

#include "math.h"

/**
 * @brief      :符号函数，x大于0时返回1，x小于0时返回-1，x等于0时返回0
 * @attention  :无
 * @return  {int} 1  x大于0
 *          {int} -1 x小于0
 *          {int} 0  x等于0
 * @param {float} x 需要判断的变量
 */
int sgn(float x) {
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
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
 * @param {float} x1 误差输入信号
 * @param {float} x2 误差的微分信号
 * @param {float} r 速度因子
 * @param {float} h 滤波因子
 */
float fst(float x1, float x2, float r, float h) {
    float d, d0, y, a0, a;
    d = r * h;
    d0 = d * h;
    y = x1 + h * x2;
    a0 = sqrt(d * d + 8 * r * fabs(y));
    if (fabs(y) > d0) {
        a = x2 + 0.5 * (a0 - d);
    } else {
        a = x2 + y / h;
    }
    if (fabs(a) <= d) {
        return (-r * a / d);
    } else {
        return (-r * sgn(a));
    }
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
    adrc_data->prog.v1 = last_v1 + adrc_data->td.h * last_v2;
    adrc_data->prog.v2 = last_v2 + adrc_data->td.h * fst(last_v1 - adrc_data->prog.v0, last_v2, adrc_data->td.r, adrc_data->td.h0);
}
/**
 * @brief      : 非线性反馈
 * @attention  : 无
 * @return  {*}
 * @param {ADRC_t*} adrc_data adrc数据结构体
 */
void NLSEFFunction(ADRC_t* adrc_data) {
    adrc_data->prog.u0 = adrc_data->nlsef.Kp * fal(adrc_data->prog.e1, adrc_data->nlsef.alpha1, adrc_data->nlsef.delta) +
                         adrc_data->nlsef.Kd * fal(adrc_data->prog.e2, adrc_data->nlsef.alpha2, adrc_data->nlsef.delta);
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
    epsilon = last_z1 - adrc_data->prog.y;
    adrc_data->prog.z1 = last_z1 + adrc_data->td.h * (last_z2 - adrc_data->nlsef.Kp * epsilon);
    adrc_data->prog.z2 = last_z2 + adrc_data->td.h * (last_z3 - adrc_data->nlsef.Kd * fal(epsilon, adrc_data->nlsef.alpha1, adrc_data->nlsef.delta) + adrc_data->eso.b * adrc_data->prog.u);
    adrc_data->prog.z3 = last_z3 - adrc_data->td.h * adrc_data->eso.beta3 * fal(epsilon, adrc_data->nlsef.alpha2, adrc_data->nlsef.delta);
}

/**
 * @brief      : 输入函数
 * @attention  : 无
 * @return  {*}
 * @param {ADRC_t*} adrc_data adrc数据结构体
 * @param {float} v0 预期输入信号
 * @param {float} y 反馈信号
 */
void GETEXINFO(ADRC_t* adrc_data, float v0, float y) {
    adrc_data->prog.v0 = v0;
    adrc_data->prog.y = y;
}

/**
 * @brief      : ADRC总函数
 * @attention  : 无
 * @return  {*}
 * @param {ADRC_t*} adrc_data adrc数据结构体
 * @param {float} v0 预期输入信号
 * @param {float} y 反馈信号
 */
void ADRCFunction(ADRC_t* adrc_data, float v0, float y) {
    GETEXINFO(adrc_data, v0, y);
    TDFunction(adrc_data);
    adrc_data->prog.e1 = adrc_data->prog.v1 - adrc_data->prog.z1;
    adrc_data->prog.e2 = adrc_data->prog.v2 - adrc_data->prog.z2;
    NLSEFFunction(adrc_data);
    adrc_data->prog.u = adrc_data->prog.u0 - adrc_data->prog.z3 / adrc_data->eso.b;
    ESOFunction(adrc_data);
}

/**
 * @brief      : ADRC结构体初始化
 * @attention  : 调参说明在adrc.h中，切勿自行更改参数
 * @return  {*}
 */
void ADRCInit(ADRC_t* adrc_data,
              float r, float h, float h0,
              float Kp, float Kd, float alpha1, float alpha2, float delta,
              float beta1, float beta2, float beta3, float b) {
    adrc_data->td.r = r;
    adrc_data->td.h = h;
    adrc_data->td.h0 = h0;
    adrc_data->nlsef.Kp = Kp;
    adrc_data->nlsef.Kd = Kd;
    adrc_data->nlsef.alpha1 = alpha1;
    adrc_data->nlsef.alpha2 = alpha2;
    adrc_data->nlsef.delta = delta;
    adrc_data->eso.beta1 = beta1;
    adrc_data->eso.beta2 = beta2;
    adrc_data->eso.beta3 = beta3;
    adrc_data->eso.b = b;
}