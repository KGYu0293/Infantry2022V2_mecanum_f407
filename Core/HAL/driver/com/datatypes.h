#ifndef _DATATYPES_H
#define _DATATYPES_H
#include "stdint.h"
#pragma pack(1)
typedef struct _pc_com
{
    uint8_t start_flag;
    float x;
    float y;
    float z;
    uint8_t end_flag;
} pc_com;

void Data_init(pc_com* obj);
void Warp_To_Array(pc_com* obj_data,uint8_t* array_buf);
void Warp_To_Obj(uint8_t* array_buf,pc_com* obj_data);

#pragma pack()
#endif