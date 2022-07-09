#ifndef _MRAC_H_
#define _MRAC_H_

#pragma pack(1)
typedef struct {
    float x1;
    float x2;
    float x;
    float r;
    float h;
    float aim;
    float dt;
} MTD_t;


typedef struct Mrac_config_t{
    float k1;
    float k2;
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;
    float output_max;
    float integrator_ki;
    float integrator_sum_err_max;
}Mrac_config;

typedef struct mrac_2d_t{
    Mrac_config config;
    float x1_ref;
    float x1_fdb;
    float x2_ref;
    float x2_fdb;
    float x1_err;
    float x2_err;
    float gamma1;
    float gamma2;
    float gamma3;
    float gamma4;
    float output;
    float dt;
    MTD_t x1_td;
    MTD_t x2_td;

    // struct
    // {
    //     float ki;
    //     float sum_err;
    //     float sum_err_max;
    // } integrator;
    float integrator_sum_err;
} mrac_2d;

#pragma pack()

void mrac_Init(mrac_2d* mrac,Mrac_config* config);
void mrac_2d_calc(mrac_2d *mrac, float ref, float x1_fdb, float x2_fdb, unsigned char enable);
#endif

/**
 * @brief          : mrac调参方法说明
 * @Note           : 为该算法调参顺序的较优解
 * @Copyright(c)   : 哈尔滨工业大学（深圳）南工骁鹰机器人队版权所有 Critical HIT copyrighted
 * 
 * 正文
 * step1:准备
 * 在未调试时，将config中outputmax按需给参，其余量均给0. 此时上电电机不输出力矩
 * 
 * step2:开始debug
 * 进入debug，逐渐增大k1，k2，直至有一定的控制效果（如pitch轴基本抬至水平），不要给太大
 * 若出现电机力矩方向为预想的相反方向，可将MOTOR_OUTPUT_REVERSE改为NORMAL
 * 
 * step3:辨识alpha2
 * (1)如果是不受重力影响的yaw，alpha2 = 0，下一步。
 * (2)如果是p轴，在本step中一直保持其指向水平不变，
 * 通过debug窗口给gamma2一个值，等待alpha2积分至稳定值。
 * 得到稳定值并记录 即为应该输入进config的值。
 * 将gamma2置为0，alpha2保持稳定值不变，进行下一步
 * 
 * step4:辨识alpha3
 * (1)如果是不受重力影响的yaw，alpha3 = 0，下一步。
 * (2)如果是p轴，控制其与水平方向成一定角度，
 * 通过debug窗口给gamma3一个值，等待alpha3积分至稳定值。
 * 得到稳定值并记录 即为应该输入进config的值。
 * 将gamma3置为0，alpha3保持稳定值不变，进行下一步
 * 
 * step5:辨识alpha4
 * 通过debug窗口给gamma4一个值，
 * 控制该轴运动，等待alpha4积分至稳定值
 * 注：该轴不动的时候不会积分
 * 记录稳定值，gamma4置零，下一步
 * 
 * step6:调k2
 * step7:调k1
 * 调k2类似于调速度环kp，调k1类似于调位置环kp。
 * 要求也类似，要看ref/fdb曲线的响应、跟随等
 * 
 * step8:辨识alpha1
 * 类似辨识alpha4,仅在运动时会积分。
 * alpha1不一定会收敛，如果辨识不成功可以考虑使用经验值alpha = 1.
 * 
 * step9:结束
 * 将上述所得的各个值输入config。
 * 
 * 其他说明：
 * 在辨识各alpha的积分过程中，积分接近稳定时对应gamma应给的较小，否则可能会造成收敛不准或者难收敛
 * 在辨识各alpha的积分过程中，非对应的其他alpha应为0。即不应出现多个gamma均不为0的情况
 * 在辨识各alpha的顺序不要替换。对step靠后的alpha辨识时，step靠前的alpha应该已经是调好的值
 * 调参过程可以中断下次接着参数调，只需要将已经调好的部分参数记录好即可，包括alpha/k等，不包括gamma
 * gamma是为了辨识alpha的，实际调好后gamma全为0，alpha为辨识得到的值
 */