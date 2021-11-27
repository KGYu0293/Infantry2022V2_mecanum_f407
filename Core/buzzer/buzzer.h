#ifndef _BUZZER_H
#define _BUZZER_H
#include "tim.h"
typedef struct _buzzer
{
    TIM_HandleTypeDef* BUZZER_PWM_BASE;
    uint16_t BUZZER_PWM_CHANNEL;
    uint16_t finished;
    uint16_t* music;
    uint16_t len;
    uint16_t count;
}buzzer;

extern volatile buzzer internal_buzzer;
//预定义的一些音乐
extern uint16_t music1[];
extern uint16_t music2[];
extern uint16_t music3[];
extern uint16_t music4[];
extern uint16_t music5[];
extern uint16_t music6[];

void Buzzer_Init(buzzer* obj,uint16_t* _music,uint16_t _len);
void Buzzer_Update(buzzer* obj);

#endif