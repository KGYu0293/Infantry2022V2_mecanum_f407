#ifndef _BSP_CAN_H
#define _BSP_CAN_H
#include "stdint.h"

typedef void (*can_rx_callback)(uint8_t can_id, uint32_t identifier, uint8_t* data,
                                uint32_t len);

void BSP_CAN_Init();
void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter);
void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter);
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t* data,
                  uint32_t len);
void BSP_CAN_RegisterRxCallback(uint8_t can_id, can_rx_callback func);
#endif