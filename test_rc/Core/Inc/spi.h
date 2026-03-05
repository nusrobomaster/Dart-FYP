#ifndef SPI_H
#define SPI_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void SPI_GPIO_Init(void);
void SPI_WriteByte(uint8_t data);
uint8_t SPI_ReadByte(void);

#endif
