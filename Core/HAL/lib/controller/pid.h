#ifndef PID_H_
#define PID_H_

#include <stdint.h>

enum PID_Mode_e { PID_POSITION = 0, PID_DELTA, PID_COMP_POSITION };

#pragma pack(1)
typedef struct PID_config_t {
    float KP;
    float KI;
    float KD;
    float KP_fine;
    float range_rough;
    float range_fine;
    float error_max;
    float outputMax;
    float compensation;
    float error_preload;
    enum PID_Mode_e PID_Mode;
} pid_config;

typedef struct PID_t {
    struct PID_config_t config;
    float error[3];
    float error_sum;
    float fdb;
    float ref;
    float output;
    float output_unlimited; // 经outputMax限制前的原始输出
    float error_delta;
} pid;
#pragma pack()

void PID_Init(struct PID_t* pid, struct PID_config_t* config);
void PID_Calc(struct PID_t* pid);
void PID_SetConfig_Pos(struct PID_config_t* obj, float kp, float ki, float kd, float errormax, float outputmax);
// void PID_SetConfig_DELTA(struct PID_config_t* obj, float kp, float ki, float kd, float errormax, float outputmax);
void PID_SetConfig_Comp(struct PID_config_t* obj, float kp_rough, float kp_fine, float ki, float kd, float rangerough, float rangefine, float compensation, float errorpreload, float errormax,
                        float outputmax);
#endif
