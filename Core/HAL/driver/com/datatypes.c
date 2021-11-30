#include "datatypes.h"
#include "string.h"

void Data_init(pc_com* obj){
    obj->start_flag = 's';
    obj->end_flag = 'e';
}
void Warp_To_Array(pc_com* obj_data,uint8_t* array_buf){
    memcpy(array_buf,obj_data,sizeof(pc_com));
}
void Warp_To_Obj(uint8_t* array_buf,pc_com* obj_data){
    memcpy(obj_data,array_buf,sizeof(pc_com));
}