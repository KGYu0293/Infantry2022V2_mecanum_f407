#ifndef _HAL_RANDOM_H
#define _HAL_RANDOM_H
#include "stdint.h"

uint32_t GetRand_Int();
float GetRand_Float();
float GetRand_Range(float rand_l,float rand_r);
#endif