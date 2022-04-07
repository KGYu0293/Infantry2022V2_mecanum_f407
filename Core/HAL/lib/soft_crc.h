#ifndef _SOFT_CRC_H_
#define _SOFT_CRC_H_

#define CRC16_CCITT 0x1021
#define CRC8_MAXIM 0x31
#define CRC_START_MODBUS_16 0xFFFF
#define CRC_START_MODBUS_8 0xFF

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

void CRC16_Init(uint16_t poly);
uint16_t CRC16_Modbus_calc(uint8_t* data, uint32_t num_bytes);

void CRC8_Init(uint8_t poly);
uint8_t CRC8_Modbus_calc(uint8_t* data, uint32_t num_bytes);
#ifdef __cplusplus
}
#endif

#endif