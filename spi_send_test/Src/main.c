//
// Created by Oguz Kagan YAGLIOGLU on 9/22/2022.
// www.oguzkagan.xyz
//

#include "string.h"
#include "stm32f103xx.h"


void softDelay() {
    for (uint32_t i = 0; i < 500000; ++i) asm("nop");
}

/**
 * SPI2
 * PB15 -> MOSI
 * PB14 -> MISO
 * PB13 -> SCK
 * PB12 -> NSS
 * */

void SPI_GPIOInit() {
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_MAX_SPEED_50MHZ;
    SPIPins.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_AF_PP;

    GPIO_PeriClockControl(GPIOB, ENABLE);

    //Serial CLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    GPIO_Init(&SPIPins);

    //MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
    GPIO_Init(&SPIPins);

    //NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
    GPIO_Init(&SPIPins);

    //MISO
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    SPIPins.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_IN_F;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
    GPIO_Init(&SPIPins);
}

void SPI2_Init() {
    SPI_Handle_t SPI2Handle;
    SPI2Handle.pSPI = SPI2;
    SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BusConfig_FD;
    SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DeviceMode_MASTER;
    SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV_2;
    SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;
    SPI_PeriClockControl(SPI2Handle.pSPI, ENABLE);
    SPI_Init(&SPI2Handle);
    SPI_PeriControl(SPI2Handle.pSPI, ENABLE);
    SPI_SSIConfig(SPI2Handle.pSPI, ENABLE);
    SPI2Handle.pSPI->SPI_CR1 |= (1 << SPI_CR1_SPE);
}


int main() {
    SPI_GPIOInit();
    SPI2_Init();

    char buffer[] = "Hello World\r\n";

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        SPI_SendData_blocking(SPI2, (uint8_t *) buffer, strlen(buffer));
        softDelay();
    }
#pragma clang diagnostic pop
    return 0;
}