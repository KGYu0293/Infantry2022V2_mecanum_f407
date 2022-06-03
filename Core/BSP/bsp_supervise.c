#include "bsp_supervise.h"
#include "main.h"
#include "string.h"
FrameRate task_frameRate;
void FrameRateStatistics(FPS_t *FPS) {
    FPS->count++;
    FPS->now = HAL_GetTick();
    int duration = FPS->now - FPS->last;
    if (duration >= 1000) {
        FPS->FPS = FPS->count * 1000 / duration;
        FPS->count = 0;
        FPS->last = FPS->now;
    }
}