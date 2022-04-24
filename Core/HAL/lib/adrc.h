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
    adrc.GETEXINFO(); //获取fdb和ref信息
    adrc.ADRCFunction(); //进行算法
    output = adrc.prog.u; //获取输出，输出到电机

    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!调参指南!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    跟踪微分器调参：
    TD_t:{
        r: 为速度因子，r越大，则跟踪微分器对指令跟踪的越快（相位滞后越小，幅值衰减越少），
                微分出来的值滞后也越小，即越接近真实值，但也会放大噪声，所以可以尽量大，直到看到高频振荡。参考值：1000000
        h: 为步长，一般取系统运算周期即可，当freertos设置为1ms时，取0.001即可
        h0: 为滤波因子，一般取h的两倍
    }
    NLSEF_t:{
        根据采样步长确定，代码中和运算周期保持一致，一般为0.001s
        beta1:  取1/h
        beta2:  取1/(3*h*h)
        alpha1: 取0.5固定参数
        alpha2: 取0.25固定参数
        delta:  取0.01固定参数
    }
    ESO_t:{
        beta3:  取1/(20*h*h*h)
        b:      取5固定参数
    }
*/

// ADRC运算的时候会出现的过程参数
typedef struct PROG_t {
    float v0;  //输入预期信号
    float v1;  //过渡过程之后的跟踪信号
    float v2;  //过渡过程之后的跟踪信号的微分(v1的微分)
    float e1;  //误差信号1
    float e2;  //误差信号2
    float u0;  //非线性反馈输出的信号
    float u;   //结合ESO后的输出信号
    float z1;  // ESO输出信号1
    float z2;  // ESO输出信号2
    float z3;  // ESO输出信号3
    float w;   //噪声干扰信号
    float y;   //最终输出
} PROG_t;
//跟踪微分器的参数
typedef struct TD_t {
    float r;
    float h;
    float h0;
} TD_t;
//非线性反馈的参数
typedef struct NLSEF_t {
    float beta1;
    float beta2;
    float alpha1;
    float alpha2;
    float delta;
} NLSEF_t;
//扩张观测器的参数，有些参数是复用非线性反馈和跟踪微分器的参数的，所以不再单独列出
typedef struct ESO_t {
    float b;
    float beta3;
} ESO_t;
// ADRC总结构体
typedef struct ADRC_t {
    PROG_t prog;
    TD_t td;
    NLSEF_t nlsef;
    ESO_t eso;
} ADRC_t;

int sgn(float x);
float fst(float x1, float x2, float r, float h);
float fal(float e, float a, float delta);
void TDFunction(ADRC_t* adrc_data);
void NLSEFFunction(ADRC_t* adrc_data);
void ESOFunction(ADRC_t* adrc_data);
void GETEXINFO(ADRC_t* adrc_data, float v0, float y);
void ADRCFunction(ADRC_t* adrc_data, float v0, float y);

#endif