#ifndef _BUZZER_H
#define _BUZZER_H
#include "stdint.h"
#include "stdlib.h"

#pragma pack(1)
typedef struct buzzer_config_t{
    uint16_t* music;
    uint16_t len;
    uint8_t bsp_pwm_index;
} buzzer_config;

typedef struct buzzer_t {
    uint16_t finished;
    uint16_t count;
    uint16_t started;
    buzzer_config config;
} buzzer;
#pragma pack()

//预定义的一些音乐
extern uint16_t music1[];
extern uint16_t music2[];
extern uint16_t music3[];
extern uint16_t music4[];
extern uint16_t music5[];
extern uint16_t music6[];
extern uint16_t* musics[];
extern uint16_t music_lens[];
// buzzer* ()
buzzer* Buzzer_Create(buzzer_config* config);
// void Buzzer_Update(buzzer* obj);
void Buzzer_Driver_Init();
void Buzzer_Loop();
void Buzzer_Start(buzzer* obj);
#endif