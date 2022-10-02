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

/**
 * @fn          SPI_Init
 *
 * @brief       This function initializes the given SPI peripheral
 *
 * @param[in]   pSPIHandle
 *
 * @return      none
 *
 * @note        none
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @fn          SPI_DeInit
 *
 * @brief       This function deinitializes the given SPI peripheral
 *
 * @param[in]   SPI_RegDef_t base address of the SPI peripheral for the given port
 *
 * @return      none
 *
 * @note        none
 * */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @fn          SPI_PeriControl
 *
 * @brief       This function enables or disables the given SPI peripheral
 *
 * @param[in]   pSPIx    base address of the SPI peripheral
 * @param[in]   EnOrDi   ENABLE or DISABLE macros
 *
 * @return      none
 *
 * @note        none
 * */
void SPI_PeriControl(SPI_RegDef_t *pSPIx, bool EnOrDi);

/**
 * @fn          SPI_PeriClockControl
 *
 * @brief       This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]   pSPIx    base address of the SPI peripheral
 * @param[in]   EnOrDi   ENABLE or DISABLE macros
 *
 * @return      none
 *
 * @note        none
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, bool EnOrDi);

/**
 * @fn          SPI_SSIConfig
 *
 * @brief       This function enables or disables SSI for the given SPI peripheral
 *
 * @param[in]   pSPIx    base address of the SPI peripheral
 * @param[in]   EnOrDi   ENABLE or DISABLE macros
 *
 * @return      none
 *
 * @note        none
 * */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, bool EnOrDi);


/**
 * @fn          SPI_SendData_blocking
 *
 * @brief       Sends len bytes from pTxBuffer
 *
 * @param[in]   pSPIx       base address of the SPI peripheral
 * @param[in]   pTxBuffer   data buffer to send
 * @param[in]   len         data buffer length
 *
 * @return      none
 *
 * @note        blocks until all data is transferred
 * */
void SPI_SendData_blocking(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/**
 * @fn          SPI_ReceiveData_blocking
 *
 * @brief       Reads len bytes from SPI to pRxBuffer
 *
 * @param[in]   pSPIx       base address of the SPI peripheral
 * @param[in]   pRxBuffer   data buffer to write incoming data
 * @param[in]   len         data buffer length
 *
 * @return      none
 *
 * @note        blocks until all data is transferred
 * */
void SPI_ReceiveData_blocking(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

#endif //STM32F1XX_DRIVERS_STM32F103XX_SPI_DRIVER_H
