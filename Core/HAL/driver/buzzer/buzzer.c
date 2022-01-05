#include "buzzer.h"
#include "string.h"
#include "stdlib.h"
#define L_Do 3822
#define L_Re 3405
#define L_Mi 3033
#define L_Fa 2863
#define L_So 2551
#define L_La 2272
#define L_Xi 2024
#define M_Do 1911
#define M_Re 1702
#define M_Mi 1526
#define M_Fa 1431
#define M_So 1275
#define M_La 1136
#define M_Xi 1012
#define H_Do 955
#define H_Re 851
#define H_Mi 758
#define H_Fa 715
#define H_So 637
#define H_La 568
#define H_Xi 506

uint16_t music1[8] = {M_Re, M_Mi, M_So, H_Do, 0, M_La, H_Do, H_Do};

uint16_t music2[14] = {H_Do, 0,    M_So, 0,    M_Mi, 0,    M_Xi,
                       M_Xi, M_Xi, M_Xi, M_La, 0,    M_Xi, H_Do};

uint16_t music3[23] = {L_So, 0, M_Do, 0, 0, M_Mi, M_Re, 0, M_Do, 0,    0,   0,
                       M_Mi, 0, M_So, 0, 0, M_La, M_Xi, 0, H_Do, H_Do, H_Do};

uint16_t music4[63] = {M_Do, 0,    M_Do, M_Mi, M_Mi, M_Mi, M_Mi, 0,    M_Do,
                       0,    M_Do, M_Mi, M_Mi, M_Mi, M_Mi, 0,    M_La, 0,
                       M_La, 0,    M_So, 0,    M_La, 0,    M_So, M_Do, 0,
                       M_Mi, M_Mi, 0,    0,    H_Do, 0,    M_La, M_La, M_So,
                       0,    M_La, 0,    0,    M_So, 0,    M_Do, M_Re, M_Re,
                       0,    0,    M_Xi, M_Xi, M_Xi, M_Xi, M_Xi, 0,    M_Xi,
                       0,    M_So, M_Mi, 0,    M_So, M_So, M_So};

uint16_t music5[62] = {H_Do, 0,    M_Xi, 0,    M_So, 0,    M_So, 0,    M_La,
                       1145, M_La, 0,    M_So, 0,    M_So, 0,    M_La, 1145,
                       M_La, 0,    M_So, 0,    M_So, 0,    M_La, 1145, M_La,
                       0,    M_La, 0,    M_La, M_La, M_Xi, M_Xi, H_Do, H_Do,
                       H_Re, H_Re, M_Xi, M_Xi, M_Xi, M_Xi, M_So, M_So, M_So,
                       M_So, H_Fa, H_Fa, H_Fa, H_Fa, H_Re, H_Re, H_Re, 0,
                       H_Re, H_Re, 0,    H_Re, H_Re, H_Re, H_Mi, H_Mi};

uint16_t music6[56] = {
    H_Do, H_Do, H_Do, 0,    H_Do, 0,    M_Xi, H_Do, H_Do, 0,    H_Re, 0,
    H_Mi, 0,    H_Fa, 0,    H_Mi, H_Mi, 0,    H_Mi, H_Mi, 0,    M_Xi, 0,
    M_Xi, M_Xi, M_Xi, 0,    0,    M_La, M_La, M_La, 0,    M_La, 0,    M_So,
    M_La, M_La, 0,    H_Fa, 0,    H_Mi, 0,    H_Re, 0,    H_Re, H_Re, 0,
    H_Do, H_Do, 0,    H_Re, 0,    H_Mi, H_Mi, H_Mi};

uint16_t music_nxt[40] = {M_Do, M_Do, M_Do, M_Do, M_Do, M_Do, M_Do, M_Do,
                          M_La, M_La, M_La, M_La, M_La, M_La, M_La, M_Fa,
                          M_Fa, M_Fa, M_Fa, M_Fa, M_Do, M_Do, M_Do, M_Do,
                          M_Do, M_Do, M_Do, M_Do, H_Do, H_Do, H_Do, H_Do,
                          H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do};

uint16_t* musics[] = {music1,music2,music3,music4,music5,music6};
uint16_t music_lens[] = {8,14,23,63,62,56};
buzzer *Buzzer_Create(buzzer_config* config){
    buzzer* obj = (buzzer*) malloc(sizeof(buzzer));
    Buzzer_Init(obj,config);
    return obj;
}

void Buzzer_Init(buzzer *obj, buzzer_config* config) {
    obj->finished = 0;
    obj->count = 0;
    obj->config = *config;
    //HAL_TIM_PWM_Start(obj->BUZZER_PWM_BASE, obj->BUZZER_PWM_CHANNEL);
    BSP_PWM_Start(obj->config.bsp_pwm_index);
}

void Buzzer_Update(buzzer *obj) {
    if (obj->finished) {
        if(obj->finished == 1){
            BSP_PWM_Stop(obj->config.bsp_pwm_index);
            obj->finished++;
        }
        return;
    }
    //__HAL_TIM_SetAutoreload(obj->BUZZER_PWM_BASE, obj->music[(int)obj->count]);
    BSP_PWM_SetARR(obj->config.bsp_pwm_index,obj->config.music[(int)obj->count]);
    if (obj->config.music[obj->count] != 0) {
        // __HAL_TIM_SetCompare(obj->BUZZER_PWM_BASE, obj->BUZZER_PWM_CHANNEL,
        //                      obj->music[obj->count] / 2);
        BSP_PWM_SetCCR(obj->config.bsp_pwm_index,obj->config.music[obj->count] / 2);
    } else {
        // __HAL_TIM_SetCompare(obj->BUZZER_PWM_BASE, obj->BUZZER_PWM_CHANNEL,
        //                      obj->music[obj->count]);
        BSP_PWM_SetCCR(obj->config.bsp_pwm_index,obj->config.music[obj->count]);
    }
    obj->count++;
    if (obj->count == obj->config.len) {
        obj->finished = 1;
        // HAL_TIM_PWM_Stop(obj->BUZZER_PWM_BASE, obj->BUZZER_PWM_CHANNEL);
        // BSP_PWM_Stop(obj->config.bsp_pwm_index);
    }
}