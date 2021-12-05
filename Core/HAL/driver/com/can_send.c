#include "can_send.h"

#include "bsp_can.h"
#include "crc16.h"
#include "cvector.h"


can_send* CanSend_Create(can_send_config* config) {
    can_send* obj = (can_send*)malloc(sizeof(can_send));
    obj->config = *config;
    obj->buf_len = obj->config.data_len + 7;
    obj->txbuf = (uint8_t*)malloc(obj->buf_len);
    obj->txbuf[0] = 's';
    obj->txbuf[1] = obj->config.data_id;
    obj->txbuf[2] = obj->config.data_type;
    obj->txbuf[3] = obj->config.data_len;
    obj->txbuf[obj->buf_len - 1] = 'e';
    return obj;
}
void CanSend_Send(can_send* obj, uint8_t* data) {
    obj->txbuf[0] = 's';
    memcpy(obj->txbuf + 4, data, obj->config.data_len);
    uint16_t crc_now = CRC16_Modbus_calc(obj->txbuf, obj->config.data_len + 3);
    memcpy(obj->txbuf + 4 + obj->config.data_len, &crc_now, 2);
    for (uint32_t idx = 0; idx < obj->buf_len; idx += 8) {
        uint32_t dlc = obj->buf_len - idx >= 8 ? 8 : obj->buf_len - idx;
        BSP_CAN_Send(obj->config.bsp_can_index, obj->config.can_identifier, obj->txbuf + idx, dlc);
    }
}