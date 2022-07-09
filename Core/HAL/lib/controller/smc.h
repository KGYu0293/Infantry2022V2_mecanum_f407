#ifndef _SMC_H_
#define _SMC_H_

typedef float (*ReachingLaw)(float data);

#pragma pack(1)
typedef struct Smc_config_t {
    float kp;  // 常规滑动模态面系数
    float kd;
    float kmax;                // 扰动增益系数
    ReachingLaw reaching_law;  // 趋近律
    float outputMax;           // 最大输出限幅
} Smc_config;

typedef struct Smc_t {
    Smc_config config;
    float error[2];
    float ref;
    float fdb;
    float output;
} Smc;
#pragma pack()

void SMC_Init(Smc* smc, Smc_config* config);
void SMC_Calc(Smc* smc);
void SMC_SetConfig(Smc_config* config, float kp, float kd, float kmax, ReachingLaw reaching_law, float outputMax);

// 趋近律
float ReachingLaw_sgn(float error);
float ReachingLaw_sqrt(float error);
float ReachingLaw_square(float error);

#endif