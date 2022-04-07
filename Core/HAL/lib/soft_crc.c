#include "soft_crc.h"

uint16_t modbus_tab16[256];
uint8_t modbus_tab16_init;
uint8_t modbus_tab8[256];
uint8_t modbus_tab8_init;

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

void CRC16_Init(uint16_t poly) {
    uint16_t table_poly = bit_reverse_u16(poly);
    for (uint16_t i = 0; i < 256; ++i) {
        uint16_t crc = 0, c = i;
        for (uint16_t j = 0; j < 8; ++j) {
            if ((crc ^ c) & 0x0001) {
                crc = (crc >> 1) ^ table_poly;
            } else
                crc = crc >> 1;
            c = c >> 1;
        }
        modbus_tab16[i] = crc;
    }
    modbus_tab16_init = 1;
}

void CRC8_Init(uint8_t poly) {
    uint8_t table_poly = bit_reverse_u8(poly);
    for (uint8_t i = 0; i < 256; ++i) {
        uint8_t crc = 0, c = i;
        for (uint8_t j = 0; j < 8; ++j) {
            if ((crc ^ c) & 0x01) {
                crc = (crc >> 1) ^ table_poly;
            } else {
                crc = crc >> 1;
            }
            c = c >> 1;
        }
        modbus_tab8[i] = crc;
    }
    modbus_tab8_init = 1;
}

uint16_t CRC16_Modbus_calc(uint8_t* data, uint32_t num_bytes) {
    uint16_t crc = CRC_START_MODBUS_16;
    while (num_bytes--) crc = (crc >> 8) ^ modbus_tab16[(crc ^ *data++) & 0xff];
    return crc;
}

uint8_t CRC8_Modbus_calc(uint8_t* data, uint32_t num_bytes) {
    uint8_t crc = CRC_START_MODBUS_8;
    while (num_bytes--) crc = modbus_tab8[(crc ^ *data++)];
    return crc;
}