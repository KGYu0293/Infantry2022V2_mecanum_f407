#include "smc.h"

#include "arm_math.h"
#include "common.h"

// sliding mode control 滑膜控制

void Smc_Init(Smc* smc, Smc_config* config) { smc->config = *config; }

void Smc_SetConfig(Smc_config* config, float kp, float kd, float kmax, ReachingLaw reaching_law, float outputMax) {
    config->kp = kp;
    config->kd = kd;
    config->kmax = kmax;
    config->reaching_law = reaching_law;
    config->outputMax = outputMax;
}

void Smc_Calc(Smc* smc) {
    smc->error[1] = smc->error[0];
    smc->error[0] = smc->ref - smc->fdb;

    float u;
    u = smc->config.reaching_law(smc->error[0]);

    smc->output = smc->config.kp * smc->error[0]                      // 经典滑膜面选择
                  + smc->config.kd * (smc->error[0] - smc->error[1])  // kd
                  + smc->config.kmax * u;                             // 趋近律 整项要能大于扰动的最大值
    if (smc->output < -smc->config.outputMax) smc->output = -smc->config.outputMax;
    if (smc->output > smc->config.outputMax) smc->output = smc->config.outputMax;
}

// 最简单的趋近律
// 易产生震动
float ReachingLaw_sgn(float error) { return fsgn(error); }

// 根号趋近
// 能有效减小稳态误差
float ReachingLaw_sqrt(float error) {
    float u;
    if (error > 0) {
        arm_sqrt_f32(fabs(error), &u);
    } else if (error < 0) {
        arm_sqrt_f32(fabs(error), &u);
        u *= -1;
    } else
        u = 0;

    return u;
}

float ReachingLaw_square(float error) { return (error * error + 0.1) * fsgn(error); }