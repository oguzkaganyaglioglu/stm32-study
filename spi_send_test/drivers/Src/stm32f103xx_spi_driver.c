//
// Created by Oguz Kagan YAGLIOGLU on 10/1/2022.
// www.oguzkagan.xyz
//

#include "stm32f103xx_spi_driver.h"

void SPI_Init(SPI_Handle_t *pSPIHandle) {
    uint32_t tempReg = 0;

    // configure the device mode
    switch (pSPIHandle->SPI_Config.SPI_BusConfig) {
        case SPI_DeviceMode_MASTER:
            tempReg |= (1 << SPI_CR1_MSTR); // set MSTR bit
            break;
        case SPI_DeviceMode_SLAVE:
        default:
            tempReg &= ~(1 << SPI_CR1_MSTR); // clear MSTR bit
            break;
    }

    // configure the bus config
    switch (pSPIHandle->SPI_Config.SPI_BusConfig) {
        case SPI_BusConfig_FD:
            // BIDI should be cleared
            tempReg &= ~(1 << SPI_CR1_BIDIMODE);
            break;
        case SPI_BusConfig_HD:
            // BIDI should be set
            tempReg |= (1 << SPI_CR1_BIDIMODE);
            break;
        case SPI_BusConfig_SIMPLEX_RXONLY:
            // BIDI mode should be cleared
            tempReg |= (1 << SPI_CR1_BIDIMODE);
            // RXONLY bit must be set
            tempReg |= (1 << SPI_CR1_RX_ONLY);
            break;
    }

    // configure the clock speed
    switch (pSPIHandle->SPI_Config.SPI_SclkSpeed) {
        case SPI_SCLK_SPEED_DIV_256:
            tempReg |= (0b111 << SPI_CR1_BR);
            break;
        case SPI_SCLK_SPEED_DIV_128:
            tempReg |= (0b110 << SPI_CR1_BR);
            break;
        case SPI_SCLK_SPEED_DIV_64:
            tempReg |= (0b101 << SPI_CR1_BR);
            break;
        case SPI_SCLK_SPEED_DIV_32:
            tempReg |= (0b100 << SPI_CR1_BR);
            break;
        case SPI_SCLK_SPEED_DIV_16:
            tempReg |= (0b011 << SPI_CR1_BR);
            break;
        case SPI_SCLK_SPEED_DIV_8:
            tempReg |= (0b010 << SPI_CR1_BR);
            break;
        case SPI_SCLK_SPEED_DIV_4:
            tempReg |= (0b001 << SPI_CR1_BR);
            break;
        case SPI_SCLK_SPEED_DIV_2:
        default:
            tempReg |= (0b000 << SPI_CR1_BR);
            break;
    }

    // configure the data frame format
    switch (pSPIHandle->SPI_Config.SPI_DFF) {
        case SPI_DFF_16BITS:
            tempReg |= (1 << SPI_CR1_DFF);
            break;
        case SPI_DFF_8BITS:
        default:
            tempReg &= ~(1 << SPI_CR1_DFF);
            break;
    }
    // configure the clock polarity
    switch (pSPIHandle->SPI_Config.SPI_CPOL) {
        case SPI_CPOL_HIGH:
            tempReg |= (1 << SPI_CR1_CPOL);
            break;
        case SPI_CPOL_LOW:
        default:
            tempReg &= ~(1 << SPI_CR1_CPOL);
    }

    // configure the clock phase
    switch (pSPIHandle->SPI_Config.SPI_CPHA) {
        case SPI_CPHA_HIGH:
            tempReg |= (1 << SPI_CR1_CPHA);
            break;
        case SPI_CPHA_LOW:
        default:
            tempReg &= ~(1 << SPI_CR1_CPHA);
    }

    // configure the software slave management
    switch (pSPIHandle->SPI_Config.SPI_CPHA) {
        case SPI_SSM_EN:
            tempReg |= (1 << SPI_CR1_SSM);
            break;
        case SPI_SSM_DI:
        default:
            tempReg &= ~(1 << SPI_CR1_SSM);
    }

    pSPIHandle->pSPI->SPI_CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    if (pSPIx == SPI1) SPI1_REG_RST();
    else if (pSPIx == SPI2) SPI2_REG_RST();
    else if (pSPIx == SPI3) SPI3_REG_RST();
}

void SPI_PeriControl(SPI_RegDef_t *pSPIx, bool EnOrDi) {
    if (EnOrDi) pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
    else pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
}

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, bool EnOrDi) {
    if (EnOrDi) {
        if (pSPIx == SPI1) SPI1_PCLK_EN();
        else if (pSPIx == SPI2) SPI2_PCLK_EN();
        else if (pSPIx == SPI3) SPI3_PCLK_EN();
    } else {
        if (pSPIx == SPI1) SPI1_PCLK_DI();
        else if (pSPIx == SPI2) SPI2_PCLK_DI();
        else if (pSPIx == SPI3) SPI3_PCLK_DI();
    }
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, bool EnOrDi) {
    if (EnOrDi) pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
    else pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
}

void SPI_SendData_blocking(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
    while (len > 0) {
        while (!((pSPIx->SPI_SR >> SPI_SR_TXE) & 1));
        if (((pSPIx->SPI_CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BITS) {
            pSPIx->SPI_DR = *((uint16_t *) pTxBuffer++);
            len--;
        } else pSPIx->SPI_DR = *((uint8_t *) pTxBuffer++);
        len--;
    }
}
