#include "bsp_random.h"
#include "rng.h"

uint32_t GetRand_Int(){
    return HAL_RNG_GetRandomNumber(&hrng);
}

float GetRand_Float(){
    return GetRand_Int() / 4294967295.0;
}

float GetRand_Range(float rand_l,float rand_r){
    return rand_l + (rand_r - rand_l) * GetRand_Float();
}