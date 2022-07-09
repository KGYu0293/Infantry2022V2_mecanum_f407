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

#pragma pack(1)
typedef struct CRC16_config_t {
    uint16_t poly;
    uint16_t crc_init;
} CRC16_config;
typedef struct CRC16_t {
    uint16_t table[256];
    uint16_t crc_init;
} CRC16;

typedef struct CRC8_config_t {
    uint8_t poly;
    uint8_t crc_init;
} CRC8_config;
typedef struct CRC8_t {
    uint8_t table[256];
    uint8_t crc_init;
} CRC8;
#pragma pack()

extern CRC16* crc16_default;
extern CRC8* crc8_default;

void soft_crc_Init();

uint16_t CRC16_Modbus_calc(uint8_t* data, uint32_t num_bytes,CRC16* crc16);
uint8_t CRC8_Modbus_calc(uint8_t* data, uint32_t num_bytes,CRC8* crc8);
#ifdef __cplusplus
}
#endif

#endif