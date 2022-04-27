#include "uart_send.h"
#include "bsp_uart.h"
#include "stdlib.h"
#include "string.h"
uart_send* UartSend_Create(uart_send_config* config) {
    uart_send* obj = (uart_send*)malloc(sizeof(uart_send));
    obj->config = *config;
    obj->buf_len = obj->config.data_len + 7;
    obj->txbuf = (uint8_t*)malloc(obj->buf_len);
    obj->txbuf[0] = 's';
    obj->txbuf[1] = (uint8_t)(obj->config.uart_identifier & 0xFF);
    obj->txbuf[2] = (uint8_t)(obj->config.uart_identifier >> 8);
    obj->txbuf[3] = obj->config.data_len;
    obj->txbuf[obj->buf_len - 1] = 'e';
    return obj;
}
void UartSend_Send(uart_send* obj, uint8_t* data) {
    memcpy(obj->txbuf + 4, data, obj->config.data_len);
    uint16_t crc_now = CRC16_Modbus_calc(obj->txbuf + 3, obj->config.data_len + 1, crc16_default);
    memcpy(obj->txbuf + 4 + obj->config.data_len, &crc_now, 2);
    BSP_UART_Send_DMA(obj->config.bsp_uart_index,obj->txbuf,obj->buf_len);
}