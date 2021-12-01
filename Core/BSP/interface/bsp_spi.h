#ifndef _BSP_SPI_H
#define _BSP_SPI_H
#include "stdint.h"

//此处可以预定义一些接口ID

#define SPI_BMI088_PORT 0

void BSP_SPI_Init();
void BSP_SPI_TransmitReceive(uint8_t spi_index, uint8_t *pTxData,
                             uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
#endif