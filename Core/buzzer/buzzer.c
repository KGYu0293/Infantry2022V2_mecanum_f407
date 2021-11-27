#include "buzzer.h"

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

buzzer internal_buzzer;

uint16_t music1[8] = {M_Re, M_Mi, M_So, H_Do, 0, M_La, H_Do, H_Do};
uint16_t music2[14] = {H_Do, 0, M_So, 0, M_Mi, 0, M_Xi, M_Xi, M_Xi, M_Xi, M_La, 0, M_Xi, H_Do};
uint16_t music3[23] = {L_So, 0, M_Do, 0, 0, M_Mi, M_Re, 0, M_Do, 0, 0, 0, M_Mi, 0, M_So, 0, 0, M_La, M_Xi, 0, H_Do, H_Do, H_Do};
uint16_t music4[63] = {M_Do, 0, M_Do, M_Mi, M_Mi, M_Mi, M_Mi, 0,
                       M_Do, 0, M_Do, M_Mi, M_Mi, M_Mi, M_Mi, 0,
                       M_La, 0, M_La, 0, M_So, 0, M_La, 0,
                       M_So, M_Do, 0, M_Mi, M_Mi, 0, 0,
                       H_Do, 0, M_La, M_La, M_So, 0, M_La, 0, 0,
                       M_So, 0, M_Do, M_Re, M_Re, 0, 0,
                       M_Xi, M_Xi, M_Xi, M_Xi, M_Xi, 0, M_Xi, 0, M_So, M_Mi, 0, M_So, M_So, M_So};
uint16_t music5[62] = {H_Do, 0, M_Xi, 0, M_So, 0,
                       M_So, 0, M_La, 1145, M_La, 0,
                       M_So, 0, M_So, 0, M_La, 1145, M_La, 0,
                       M_So, 0, M_So, 0, M_La, 1145, M_La, 0,
                       M_La, 0, M_La, M_La, M_Xi, M_Xi, H_Do, H_Do, H_Re, H_Re,
                       M_Xi, M_Xi, M_Xi, M_Xi, M_So, M_So, M_So, M_So,
                       H_Fa, H_Fa, H_Fa, H_Fa, H_Re, H_Re, H_Re, 0,
                       H_Re, H_Re, 0, H_Re, H_Re, H_Re, H_Mi, H_Mi}; //Only My Railgun
uint16_t music6[56] = {H_Do, H_Do, H_Do, 0, H_Do, 0, M_Xi, H_Do, H_Do, 0,
                       H_Re, 0, H_Mi, 0, H_Fa, 0,
                       H_Mi, H_Mi, 0, H_Mi, H_Mi, 0, M_Xi, 0, M_Xi, M_Xi, M_Xi, 0, 0,
                       M_La, M_La, M_La, 0, M_La, 0, M_So, M_La, M_La, 0,
                       H_Fa, 0, H_Mi, 0, H_Re, 0,
                       H_Re, H_Re, 0, H_Do, H_Do, 0, H_Re, 0, H_Mi, H_Mi, H_Mi};
uint16_t music_nxt[40] = {M_Do, M_Do, M_Do, M_Do, M_Do, M_Do, M_Do, M_Do,
                          M_La, M_La, M_La, M_La, M_La, M_La, M_La,
                          M_Fa, M_Fa, M_Fa, M_Fa, M_Fa,
                          M_Do, M_Do, M_Do, M_Do, M_Do, M_Do, M_Do, M_Do,
                          H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do, H_Do};


void Buzzer_Init(buzzer* obj,uint16_t* _music,uint16_t _len){
    obj->finished = 0;
    obj->music = _music;
    obj->len = _len;
    obj->count = 0;
}
void Buzzer_Update(buzzer* obj){
    if(obj->finished) return;
    // __HAL_TIM_SetCompare
    if(obj->music[obj->count] != 0){

    }
    else{

    }
    obj->count++;
}