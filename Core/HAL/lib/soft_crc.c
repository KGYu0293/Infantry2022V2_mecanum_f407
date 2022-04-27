#include "soft_crc.h"

#include <stdlib.h>

CRC16* crc16_default;
CRC8* crc8_default;

uint16_t bit_reverse_u16(uint16_t x) {
    uint16_t result = 0;
    for (int i = 0; i < 16; ++i) {
        uint16_t nowbit = (x >> i) & 1;
        result |= (nowbit << (15 - i));
    }
    return result;
}

uint8_t bit_reverse_u8(uint8_t x) {
    uint8_t result = 0;
    for (int i = 0; i < 8; ++i) {
        uint8_t nowbit = (x >> i) & 1;
        result |= (nowbit << (7 - i));
    }
    return result;
}

CRC16* CRC16_Create(CRC16_config* config) {
    CRC16* obj = (CRC16*)malloc(sizeof(CRC16));
    uint16_t table_poly = bit_reverse_u16(config->poly);
    for (uint16_t i = 0; i < 256; ++i) {
        uint16_t crc = 0, c = i;
        for (uint16_t j = 0; j < 8; ++j) {
            if ((crc ^ c) & 0x0001) {
                crc = (crc >> 1) ^ table_poly;
            } else
                crc = crc >> 1;
            c = c >> 1;
        }
        obj->table[i] = crc;
    }
    obj->crc_init = config->crc_init;
    return obj;
}

CRC8* CRC8_Create(CRC8_config* config) {
    CRC8* obj = (CRC8*)malloc(sizeof(CRC8));
    uint8_t table_poly = bit_reverse_u8(config->poly);
    for (uint16_t i = 0; i < 256; ++i) {
        uint8_t crc = 0, c = i;
        for (uint8_t j = 0; j < 8; ++j) {
            if ((crc ^ c) & 0x01) {
                crc = (crc >> 1) ^ table_poly;
            } else {
                crc = crc >> 1;
            }
            c = c >> 1;
        }
        obj->table[i] = crc;
    }
    obj->crc_init = config->crc_init;
    return obj;
}

void soft_crc_Init() {
    CRC16_config crc16_default_config;
    crc16_default_config.crc_init = CRC_START_MODBUS_16;
    crc16_default_config.poly = CRC16_CCITT;
    crc16_default = CRC16_Create(&crc16_default_config);

    CRC8_config crc8_default_config;
    crc8_default_config.crc_init = CRC_START_MODBUS_8;
    crc8_default_config.poly = CRC8_MAXIM;
    crc8_default = CRC8_Create(&crc8_default_config);
}

uint16_t CRC16_Modbus_calc(uint8_t* data, uint32_t num_bytes, CRC16* crc16) {
    uint16_t crc = crc16->crc_init;
    while (num_bytes--) crc = (crc >> 8) ^ crc16->table[(crc ^ *data++) & 0xff];
    return crc;
}

uint8_t CRC8_Modbus_calc(uint8_t* data, uint32_t num_bytes, CRC8* crc8) {
    uint8_t crc = crc8->crc_init;
    while (num_bytes--) crc = crc8->table[(crc ^ *data++)];
    return crc;
}