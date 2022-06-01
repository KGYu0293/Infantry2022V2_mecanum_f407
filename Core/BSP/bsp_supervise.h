#ifndef _SUPERVISE_H_
#define _SUPERVISE_H_

#include "stdint.h"

typedef struct {
    short count;
    short FPS;
    int now;
    int last;
} FPS_t;

typedef struct {
    FPS_t FPS_Cap;
    FPS_t FPS_RobotCMD;
    FPS_t FPS_IMU;
    FPS_t FPS_controller;
    FPS_t FPS_Judge;
    FPS_t FPS_JudgeSend;
} FrameRate;

extern FrameRate task_frameRate;
void FrameRateStatistics(FPS_t *FPS);

#endif
