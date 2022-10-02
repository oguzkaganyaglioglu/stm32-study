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

/**
 * @SPI_DeviceModes
 * */
#define SPI_DeviceMode_SLAVE        0
#define SPI_DeviceMode_MASTER       1

/**
 * @SPI_BusConfig
 * */
#define SPI_BusConfig_FD                    1   // Full Duplex
#define SPI_BusConfig_HD                    2   // Half Duplex
#define SPI_BusConfig_SIMPLEX_RXONLY        3   // Simplex RX only

/**
 * @SPI_ClkSpeed
 * */
#define SPI_SCLK_SPEED_DIV_2        0
#define SPI_SCLK_SPEED_DIV_4        1
#define SPI_SCLK_SPEED_DIV_8        2
#define SPI_SCLK_SPEED_DIV_16       3
#define SPI_SCLK_SPEED_DIV_32       4
#define SPI_SCLK_SPEED_DIV_64       5
#define SPI_SCLK_SPEED_DIV_128      6
#define SPI_SCLK_SPEED_DIV_256      7

/**
 * @SPI_DFF
 * */
#define SPI_DFF_8BITS       0
#define SPI_DFF_16BITS      1

/**
 * @SPI_CPOL
 * */
#define SPI_CPOL_LOW        0
#define SPI_CPOL_HIGH       1

/**
 * @SPI_CPHA
 * */
#define SPI_CPHA_LOW        0
#define SPI_CPHA_HIGH       1

/**
 * @SPI_SSM
 * */
#define SPI_SSM_EN          1
#define SPI_SSM_DI          0

#endif //STM32F1XX_DRIVERS_STM32F103XX_SPI_DRIVER_H
