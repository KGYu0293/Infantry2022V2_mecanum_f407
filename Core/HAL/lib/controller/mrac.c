/**
 * @file           : mrac.c
 * @brief          :
 * @Author         : lwy
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
    if (enable == 1) {
        float cosx, sinx;
        cosx = cos(x1_fdb * M_PI / 180.0f);
        sinx = sin(x1_fdb * M_PI / 180.0f);
        mrac->x1_fdb = x1_fdb;
        mrac->x2_fdb = x2_fdb;
        MTDFunction(&mrac->x1_td, ref);
        mrac->x1_ref = mrac->x1_td.x1;
        mrac->x1_err = mrac->x1_ref - mrac->x1_fdb;

        mrac->integrator.sum_err += mrac->x1_err;
        mrac->integrator.sum_err = sat(mrac->integrator.sum_err, -mrac->integrator.sum_err_max, mrac->integrator.sum_err_max);

        mrac->x2_ref = mrac->k1 * mrac->x1_err + mrac->x1_td.x2;
        mrac->x2_err = mrac->x2_ref - mrac->x2_fdb;
        MTDFunction(&mrac->x2_td, mrac->x2_ref);

        mrac->output = mrac->k2 * mrac->x2_err +
                       mrac->alpha1 * mrac->x2_td.x2 +
                       mrac->alpha2 * cosx +
                       mrac->alpha3 * sinx +
                       mrac->alpha4 * sgn_like(mrac->x2_fdb, 0.5) +
                       mrac->integrator.ki * mrac->integrator.sum_err;
        mrac->output = sat(mrac->output, -mrac->output_max, mrac->output_max);

        mrac->alpha1 += mrac->dt * mrac->gamma1 * mrac->x2_err * mrac->x2_td.x2;
        mrac->alpha2 += mrac->dt * mrac->gamma2 * mrac->x2_err * cosx;
        mrac->alpha3 += mrac->dt * mrac->gamma3 * mrac->x2_err * sinx;
        mrac->alpha4 += mrac->dt * mrac->gamma4 * mrac->x2_err * fsgn(mrac->x2_fdb);
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
    }
}