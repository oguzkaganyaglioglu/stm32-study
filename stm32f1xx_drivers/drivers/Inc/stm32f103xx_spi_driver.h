//
// Created by Oguz Kagan YAGLIOGLU on 10/1/2022.
// www.oguzkagan.xyz
//

#ifndef STM32F1XX_DRIVERS_STM32F103XX_SPI_DRIVER_H
#define STM32F1XX_DRIVERS_STM32F103XX_SPI_DRIVER_H

#include "stm32f103xx.h"

/*
 * Configuration structure for SPIx peripheral
 * */
typedef struct {
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 * */
typedef struct {
    SPI_RegDef_t *pSPI;         // base address of the SPIx peripheral
    SPI_Config_t SPI_Config;
} SPI_Handle_t;


#endif //STM32F1XX_DRIVERS_STM32F103XX_SPI_DRIVER_H
