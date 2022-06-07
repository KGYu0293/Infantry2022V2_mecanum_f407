#ifndef _ADRC_H_
#define _ADRC_H_

/*
    ADRC控制算法流程图：
                |---------------|   v1           e1 |-------------------|                                               w ↓
        v0      |               |-→-→-→-O-→-→-→-→-→-|                   |  u0           u                u          |-----------|       y
    -→-→-→-→-→-→|  TDFuntion()  |      -↑           |  NLSEFFunction()  |-→-→-O-→-→-→-→-→-→-→-→-O-→-→-→-→-→-→-→-→-→-|   object  |-→-→-→-→-→-→-→-→
                |               |       ↑        e2 |                   |    -↑                 ↓                   |-----------|       ↓
                |---------------|-→-→-→-↑-→-O-→-→-→-|-------------------|  | 1/b |            | b |                                     ↓
                                    v2  ↑  -↑                                 ↑                 ↓                                       ↓
                                        ↑   ↑                                 ↑     |-------------------|                               ↓
                                        ↑   ↑                                 ↑  z3 |                   |                               ↓
                                        ↑   ↑                                 ↑-←-←-|                   |                               ↓
                                        ↑   ↑               z2                      |   ESOFunction()   |←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←↓
                                        ↑   ↑←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←|                   |
                                        ↑                   z1                      |                   |
                                        ↑←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←-←|-------------------|

    注：
        TDFunction()：过渡过程(跟踪微分器)，由fhan最速综合函数得到
        NLSEFFunction()：非线性反馈
        ESOFunction()：扩张观测器



    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!使用说明!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    请务必按照以下形式使用adrc算法:
    ADRC_t adrc_data;                       //创建结构体
    ref = xxxxxx;                           //得到预期输入值
    fdb = xxxxxx;                           //得到反馈值
    ADRCFunction(&adrc_data, ref, fdb);     //adrc算法计算
    output = adrc_data.prog.u;              //得到输出

    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!调参指南!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    TD_t:{
        r:  速度因子，r越大，则跟踪微分器对指令跟踪的越快（相位滞后越小，幅值衰减越少），
                微分出来的值滞后也越小，即越接近真实值，但也会放大噪声，所以可以尽量大，直到看到高频振荡。参考值：16384
        h:  步长，一般取系统运算周期即可，当freertos设置为1ms时，取0.001即可。参考值：0.001
        h0: 滤波因子，一般取h的两倍。参考值：0.002
    }
    NLSEF_t:{
        Kp:     非线性反馈的比例增益，取1/h
        Kd:     非线性反馈的微分增益，取1/(3*h*h)
        alpha1: 第一个非线性因子，取0.5固定值
        alpha2: 第二个非线性因子，取0.25固定值
        delta:  滤波因子，取0.01固定值
    }
    ESO_t:{
        Ω为观测器带宽，扰动频率低时，可以用小带宽，扰动频率高时，可以用大带宽。
        比如英雄不经常在颠簸路面行驶，带宽不需要很高，Ω取1即可
        步兵经常在颠簸路面行驶，带宽需要很高，Ω取10
        beta1、beta2、beta3均为观测器参数
        beta1:  取3Ω。对于步兵，参考值：30
        beta2:  取3Ω²。对于步兵，参考值：300
        beta3:  取Ω³。对于步兵，参考值：1000
        b:      补偿系数，b越大，越能抑制系统的抖震，但也会使系统的控制效果大打折扣。参考值：5
    }

    另：关于b、Kp、Kd、Ω这四个不好调整的参数的整定策略
    1. 首先固定一个较小的b，先设定一个比较小的Kp和Kd，之后尽量选用很大的带宽Ω，直到系统发生振动
    2. 如果第一步中找不到稳定的参数，可以调大b，重新第一步
    3. 在固定好带宽之后，可以适当调大Kp和Kd，做到系统快速性的改善
*/

// ADRC运算的时候会出现的过程参数
typedef struct PROG_t {
    float ref;     //输入预期信号
    float v1;      //过渡过程之后的跟踪信号
    float v2;      //过渡过程之后的跟踪信号的微分(v1的微分)
    float e1;      //误差信号1
    float e2;      //误差信号2
    float u0;      //非线性反馈输出的信号
    float output;  //结合ESO后的输出信号
    float z1;      // ESO输出信号1
    float z2;      // ESO输出信号2
    float z3;      // ESO输出信号3
    float fdb;     //反馈信号
} PROG_t;

//跟踪微分器的参数
typedef struct TD_t {
    float r;
    float h;
    float h0;
} TD_t;

//非线性反馈的参数
typedef struct NLSEF_t {
    float Kp;
    float Kd;
    float alpha1;
    float alpha2;
    float delta;
} NLSEF_t;

//扩张观测器的参数
typedef struct ESO_t {
    float beta1;
    float beta2;
    float beta3;
    float b;
} ESO_t;

typedef struct ADRC_Config_t {
    TD_t td;
    NLSEF_t nlsef;
    ESO_t eso;
    float output_max;
} ADRC_Config_t;

// ADRC总结构体
typedef struct ADRC_t {
    PROG_t prog;
    ADRC_Config_t adrc_config;
} ADRC_t;


void ADRC_SetConfig(ADRC_Config_t* adrc_config,
                    float r, float h, float h0,
                    float Kp, float Kd, float alpha1, float alpha2, float delta,
                    float beta1, float beta2, float beta3, float b,float output_max);
void ADRC_Init(ADRC_t* adrc_data, ADRC_Config_t* adrc_config);
void ADRCFunction(ADRC_t* adrc_data);
#endif