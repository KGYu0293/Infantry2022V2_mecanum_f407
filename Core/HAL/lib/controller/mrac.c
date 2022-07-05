/**
 * @file           : mrac.c
 * @brief          :
 * @Author         : 刘文熠
 * @Date           : 2022-04-10 09:36
 * @LastEditTime   : 2022-04-28 17:34
 * @Note           :
 * @Copyright(c)   : 哈尔滨工业大学（深圳）南工骁鹰机器人队版权所有 Critical HIT copyrighted
 */

#include "mrac.h"

#include "common.h"
#include "math.h"

float sat(float x, float lower_bound, float upper_bound);
void MTDFunction(MTD_t *ptd, float aim);

float sat(float x, float lower_bound, float upper_bound) {
    if (lower_bound > upper_bound) return 0;
    if (x > upper_bound)
        return upper_bound;
    else if (x < lower_bound)
        return lower_bound;
    else
        return x;
}

void MTDFunction(MTD_t *ptd, float aim) {
    float d, d0, y, k0, k, a = 0.0f;
    ptd->aim = aim;
    ptd->x = ptd->x1 - ptd->aim;
    d = ptd->h * ptd->r;
    d0 = d * ptd->h;
    y = ptd->x + ptd->h * ptd->x2;
    k0 = (1 + sqrt(1 + 8 * fabs(y) / d0)) / 2.0f;
    k = fsgn(k0 - floor(k0)) + floor(k0);
    if (fabs(y) > d0)
        a = ptd->r * sat((1 - k / 2.0f) * fsgn(y) - (ptd->x + k * ptd->h * ptd->x2) / ((k - 1) * d0), -1.0f, 1.0f);
    else
        a = -ptd->r * sat((ptd->x2 + y / ptd->h) / d, -1.0f, 1.0f);
    ptd->x1 += ptd->dt * ptd->x2;
    ptd->x2 += ptd->dt * a;
}

/**
 * @brief MRAC calculate
 * @param mrac struct, ref, all fdb
 * @retval None
 */
void mrac_2d_calc(mrac_2d *mrac, float ref, float x1_fdb, float x2_fdb, unsigned char enable) {
    mrac->x1_fdb = x1_fdb;
    mrac->x2_fdb = x2_fdb;
    if (enable == 1) {
        float cosx, sinx;
        cosx = cos(x1_fdb * M_PI / 180.0f);
        sinx = sin(x1_fdb * M_PI / 180.0f);
        MTDFunction(&mrac->x1_td, ref);
        mrac->x1_ref = mrac->x1_td.x1;
        mrac->x1_err = mrac->x1_ref - mrac->x1_fdb;

        mrac->integrator_sum_err += mrac->x1_err;
        mrac->integrator_sum_err = sat(mrac->integrator_sum_err, -mrac->config.integrator_sum_err_max, mrac->config.integrator_sum_err_max);

        mrac->x2_ref = mrac->config.k1 * mrac->x1_err + mrac->x1_td.x2;
        mrac->x2_err = mrac->x2_ref - mrac->x2_fdb;
        MTDFunction(&mrac->x2_td, mrac->x2_ref);

        // 输出计算
        mrac->output = mrac->config.k2 * mrac->x2_err +                                       // 速度环
                       mrac->config.alpha1 * mrac->x2_td.x2 +                                 // 转动惯量补偿项
                       mrac->config.alpha2 * cosx +                                           // 重力补偿项
                       mrac->config.alpha3 * sinx +                                           // 重力补偿项
                       mrac->config.alpha4 * sgn_like(mrac->x2_fdb, 0.5) +                    // 摩擦力（阻力）补偿项
                       mrac->config.integrator_ki * mrac->integrator_sum_err;                 // 未建模阻力积分
        mrac->output = sat(mrac->output, -mrac->config.output_max, mrac->config.output_max);  // 输出限幅

        // 辨识过程 调试中给gamma得到alpha积分，调试完成后使用直接给定alpha
        mrac->config.alpha1 += mrac->dt * mrac->gamma1 * mrac->x2_err * mrac->x2_td.x2;
        mrac->config.alpha2 += mrac->dt * mrac->gamma2 * mrac->x2_err * cosx;
        mrac->config.alpha3 += mrac->dt * mrac->gamma3 * mrac->x2_err * sinx;
        mrac->config.alpha4 += mrac->dt * mrac->gamma4 * mrac->x2_err * fsgn(mrac->x2_fdb);
    } else {
        mrac->x1_td.aim = x1_fdb;
        mrac->x1_td.x1 = x1_fdb;
        mrac->x1_td.x2 = 0;
        mrac->x1_ref = x1_fdb;
        mrac->x1_err = 0;
        mrac->x2_td.aim = x2_fdb;
        mrac->x2_td.x1 = x2_fdb;
        mrac->x2_td.x2 = 0;
        mrac->x2_ref = x2_fdb;
        mrac->x2_err = 0;
        mrac->output = 0;

        mrac->integrator_sum_err = 0;
    }
}

void mrac_Init(mrac_2d *mrac, Mrac_config *config) {
    memset(mrac, 0, sizeof(mrac_2d));  // 清空内容
    mrac->config = *config;            // config修改

    // 给定参数 一般无需修改
    // 正常控制时gamma0
    mrac->gamma1 = 0;
    mrac->gamma2 = 0;
    mrac->gamma3 = 0;
    mrac->gamma4 = 0;
    // dt
    mrac->dt = 0.001f;
    // 跟踪微分器参数
    mrac->x1_td.r = 8000;
    mrac->x1_td.h = 0.001f;
    mrac->x1_td.dt = 0.001f;
    mrac->x2_td.r = 20000;
    mrac->x2_td.h = 0.001f;
    mrac->x2_td.dt = 0.001f;
    // td初始化
    mrac->x1_td.x1 = 0;
    mrac->x1_td.x2 = 0;
    mrac->x1_td.x = 0;
    mrac->x1_td.aim = 0;
    mrac->x2_td.x1 = 0;
    mrac->x2_td.x2 = 0;
    mrac->x2_td.aim = 0;
    mrac->x2_td.x = 0;
}