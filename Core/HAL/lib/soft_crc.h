#ifndef _SOFT_CRC_H_
#define _SOFT_CRC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

void CRC16_Init();
uint16_t CRC16_Modbus_calc(uint8_t* data, uint32_t num_bytes);

void CRC8_Init();
uint8_t CRC8_Modbus_calc(uint8_t* data, uint32_t num_bytes);
#ifdef __cplusplus
}
#endif

#endif