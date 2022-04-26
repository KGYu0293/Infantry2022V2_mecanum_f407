#include "radiotube.h"

#include "bsp_delay.h"
#include "bsp_gpio.h"
#include "stdlib.h"
#include "string.h"

spi_EIO *Spi_EIO_Create(spi_eio_config *config) {
    spi_EIO *obj = (spi_EIO *)malloc(sizeof(spi_EIO));
    obj->config = *config;
    obj->monitor = Monitor_Register(obj->config.lost_callback, 10, obj);
    memset(obj, 0, sizeof(spi_EIO));
    obj->Eio_tx_data_len = config->eio_tx_data_len;
    obj->Eio_rx_data_len = config->eio_rx_data_len;
    obj->EIO_Tx_Port = config->eio_tx_port;
    obj->EIO_Tx_Port = config->eio_rx_port;
    return obj;
}

void Spi_EIO_Transmit(spi_EIO *obj) {
    BSP_SPI_Transmit(obj->config.bsp_spi_index, obj->EIO_tx_data, obj->Eio_tx_data_len, 100);  //发送数据到寄存器
    bsp_delay_us(20);
    BSP_GPIO_Set(obj->EIO_Tx_Port, 0);  //将移位寄存器的数据载入到存储寄存器中
    bsp_delay_us(20);
    BSP_GPIO_Set(obj->EIO_Tx_Port, 1);  //更新输出状态（提供一个脉冲）
}

void Spi_EIO_Receive(spi_EIO *obj) {
    obj->monitor->reset(obj->monitor);
    BSP_GPIO_Set(obj->EIO_Rx_Port, 0);  //读取数据到寄存器
    bsp_delay_us(20);
    BSP_GPIO_Set(obj->EIO_Rx_Port, 1);  //停止读取并允许移位
    bsp_delay_us(20);
    BSP_SPI_Receive(obj->config.bsp_spi_index, obj->EIO_rx_data, obj->Eio_rx_data_len, 100);  //接收，提供时钟信号，从机接收到时钟信号时开始移位，并发送数据给主机
}