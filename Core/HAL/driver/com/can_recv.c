#include "can_recv.h"
#include "cvector.h"

cvector* can_recv_instances;

void CanRecv_Driver_Init(){
    can_recv_instances = cvector_create(sizeof(can_recv*));
}