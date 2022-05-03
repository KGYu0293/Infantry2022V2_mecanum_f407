#include "bsp_spi.h"

#include "bsp_def.h"
#include "spi.h"

typedef struct BSP_SPI_Typedef_t {
    SPI_HandleTypeDef *port;
} BSP_SPI_Typedef;

BSP_SPI_Typedef spi_ports[DEVICE_SPI_CNT];

void BSP_SPI_Init() {
    spi_ports[0].port = SPI_0_PORT;
}

void BSP_SPI_TransmitReceive(uint8_t spi_index, uint8_t *pTxData,
                             uint8_t *pRxData, uint16_t Size,
                             uint32_t Timeout) {
    HAL_SPI_TransmitReceive(spi_ports[spi_index].port, pTxData, pRxData, Size,
                            Timeout);
}

void BSP_SPI_Transmit(uint8_t spi_index, uint8_t *pTxData, uint16_t Size, uint32_t Timeout) {
    HAL_SPI_Transmit(spi_ports[spi_index].port, pTxData, Size, Timeout);
}

void BSP_SPI_Receive(uint8_t spi_index, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    HAL_SPI_Receive(spi_ports[spi_index].port, pRxData, Size, Timeout);
}