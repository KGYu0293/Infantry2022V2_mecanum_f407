#ifndef _BSP_SPI_H
#define _BSP_SPI_H
#include "stdint.h"

void BSP_SPI_Init();
void BSP_SPI_TransmitReceive(uint8_t spi_index, uint8_t *pTxData,
                             uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
void BSP_SPI_Transmit(uint8_t spi_index, uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
void BSP_SPI_Receive(uint8_t spi_index, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
#endif