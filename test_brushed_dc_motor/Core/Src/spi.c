#include "spi.h"

extern SPI_HandleTypeDef hspi4;

void SPI_GPIO_Init(void)
{
    /* SPI4 is initialized in MX_SPI4_Init() */
}

void SPI_WriteByte(uint8_t data)
{
    (void)HAL_SPI_Transmit(&hspi4, &data, 1U, HAL_MAX_DELAY);
}

uint8_t SPI_ReadByte(void)
{
    uint8_t tx = 0x00;
    uint8_t rx = 0x00;
    (void)HAL_SPI_TransmitReceive(&hspi4, &tx, &rx, 1U, HAL_MAX_DELAY);
    return rx;
}
