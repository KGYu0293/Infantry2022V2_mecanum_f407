#include "bsp_time.h"

#include "bsp_def.h"
#include "main.h"
uint32_t BSP_sys_time_ms() {
    return HAL_GetTick();
}