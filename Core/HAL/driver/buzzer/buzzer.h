#ifndef _BUZZER_H
#define _BUZZER_H
#include "tim.h"
typedef struct buzzer_t {
    TIM_HandleTypeDef* BUZZER_PWM_BASE;
    uint16_t BUZZER_PWM_CHANNEL;
    uint16_t finished;
    uint16_t* music;
    uint16_t len;
    uint16_t count;
} buzzer;

typedef struct buzzer_config_t{
    uint16_t* music;
    uint16_t len;
} buzzer_config;

//预定义的一些音乐
extern uint16_t music1[];
extern uint16_t music2[];
extern uint16_t music3[];
extern uint16_t music4[];
extern uint16_t music5[];
extern uint16_t music6[];

// buzzer* ()
buzzer* Buzzer_Create(buzzer_config* config);
void Buzzer_Init(buzzer* obj, buzzer_config* config);
void Buzzer_Update(buzzer* obj);

#endif