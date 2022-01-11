#ifndef _SPI_EIO_H
#define _SPI_EIO_H

#include "bsp_spi.h"
#include "monitor.h"

typedef struct _spi_eio_config_t {
    uint8_t bsp_spi_index;
    int eio_tx_data_len; //发送数据所用字节数
    int eio_rx_data_len; //接收数据所用字节数
    int eio_tx_port; //发送控制引脚
    int eio_rx_port; //接收控制引脚

    lost_callback lost_callback;  
} spi_eio_config;

typedef struct spi_EIO_t {
    int Eio_tx_data_len;
    int Eio_rx_data_len;

    int EIO_Tx_Port;
    int EIO_Rx_Port;

    uint8_t EIO_rx_data[10];
    uint8_t EIO_tx_data[10];

    spi_eio_config config;
    monitor_item* monitor;
} spi_EIO;

void Spi_EIO_Transmit(spi_EIO* obj);
void Spi_EIO_Receive(spi_EIO* obj);
spi_EIO* Spi_EIO_Create(spi_eio_config* config);

#endif
