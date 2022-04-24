/**
 * @file           : adrc.c
 * @brief          : ADRC算法实现
 * @version        : V1.0.0
 * @Author         : 李鸣航
 * @Date           : 2022-04-09 14:48
 * @LastEditTime   : 2022-04-23 12:05
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
 * @brief      :
 * @param       {*}
 * @return      {*}
 * @attention  :
 *          1.当a < 1时，函数具有小误差，大增益；大误差，小增益的特点。
 */
/**
 * @brief      :
 * @attention  :
 * @return  {*}
 *          {}
 * @param {float} e
 * @param {float} a
 * @param {float} delta
 */
float fal(float e, float a, float delta) {
    if (fabs(e) <= delta) {
        return (e / pow(delta, 1 - a));
    } else {
        return (sgn(e) * pow(fabs(e), a));
    }
}

/**
 * @brief      :
 * @param       {*}
 * @return      {*}
 * @attention  :
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
 * @brief      :
 * @param       {*}
 * @return      {*}
 * @attention  :
 */
void TDFunction(ADRC_t* adrc_data) {
    float last_v1, last_v2;
    last_v1 = adrc_data->prog.v1;
    last_v2 = adrc_data->prog.v2;
    adrc_data->prog.v1 = last_v1 + adrc_data->td.h * last_v2;
    adrc_data->prog.v2 = last_v2 + adrc_data->td.h * fst(last_v1 - adrc_data->prog.v0, last_v2, adrc_data->td.r, adrc_data->td.h0);
}

/**
 * @brief      :
 * @param       {*}
 * @return      {*}
 * @attention  :
 */
void NLSEFFunction(ADRC_t* adrc_data) {
    adrc_data->prog.u0 = adrc_data->nlsef.beta1 * fal(adrc_data->prog.e1, adrc_data->nlsef.alpha1, adrc_data->nlsef.delta) +
                         adrc_data->nlsef.beta2 * fal(adrc_data->prog.e2, adrc_data->nlsef.alpha2, adrc_data->nlsef.delta);
}

/**
 * @brief      :
 * @param       {*}
 * @return      {*}
 * @attention  :
 */
void ESOFunction(ADRC_t* adrc_data) {
    float epsilon, last_z1, last_z2, last_z3;
    last_z1 = adrc_data->prog.z1;
    last_z2 = adrc_data->prog.z2;
    last_z3 = adrc_data->prog.z3;
    epsilon = last_z1 - adrc_data->prog.y;
    adrc_data->prog.z1 = last_z1 + adrc_data->td.h * (last_z2 - adrc_data->nlsef.beta1 * epsilon);
    adrc_data->prog.z2 = last_z2 + adrc_data->td.h * (last_z3 - adrc_data->nlsef.beta2 * fal(epsilon, adrc_data->nlsef.alpha1, adrc_data->nlsef.delta) + adrc_data->eso.b * adrc_data->prog.u);
    adrc_data->prog.z3 = last_z3 - adrc_data->td.h * adrc_data->eso.beta3 * fal(epsilon, adrc_data->nlsef.alpha2, adrc_data->nlsef.delta);
}

/**
 * @brief      :
 * @param       {*}
 * @return      {*}
 * @attention  :
 */
void GETEXINFO(ADRC_t* adrc_data, float v0, float y) {
    adrc_data->prog.v0 = v0;
    adrc_data->prog.y = y;
}

/**
 * @brief      :
 * @param       {*}
 * @return      {*}
 * @attention  :
 * @param {ADRC_t*} adrc_data
 * @param {float} v0
 * @param {float} y
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