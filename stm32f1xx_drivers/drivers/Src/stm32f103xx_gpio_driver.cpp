//
// Created by Oguz Kagan YAGLIOGLU on 9/20/2022.
// www.oguzkagan.xyz
//

#include "stm32f103xx_gpio_driver.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    bool crLorH = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= GPIO_PIN_8;
    uint8_t crShiftVal = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % GPIO_PIN_8;

    // configure mode of the GPIO pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= 0b11) {
        pGPIOHandle->pGPIOx->CR[crLorH] &= ~(0b11 << (4 * crShiftVal)); // clear
        pGPIOHandle->pGPIOx->CR[crLorH] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * crShiftVal); // set
    }

    // configure cnf of the GPIO pin
    if (((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_IN) &&
        // if the cnf is reserved, configure the pin as Floating Input(reset state)
        (pGPIOHandle->GPIO_PinConfig.GPIO_PinCNF == GPIO_CNF_IN_RESERVED))
        pGPIOHandle->GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_IN_F;
    pGPIOHandle->pGPIOx->CR[crLorH] &= ~(0b11 << ((4 * crShiftVal) + 2)); // clear
    pGPIOHandle->pGPIOx->CR[crLorH] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinCNF << ((4 * crShiftVal) + 2); // set



    // configure pupd settings
    if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_IN) {
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl == GPIO_PU)
            pGPIOHandle->pGPIOx->ODR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
        else pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

}

void GPIO_DeInit(GPIO_RegDef_t *GPIOx) {
    if (GPIOx == GPIOA) GPIOA_REG_RST();
    else if (GPIOx == GPIOB) GPIOB_REG_RST();
    else if (GPIOx == GPIOC) GPIOC_REG_RST();
    else if (GPIOx == GPIOD) GPIOD_REG_RST();
    else if (GPIOx == GPIOE) GPIOE_REG_RST();
}

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, bool EnOrDi) {
    if (EnOrDi) {
        if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
        else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
        else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
        else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
        else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
    } else {
        if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
        else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
        else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
        else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
        else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
    }
}

bool GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    return (pGPIOx->IDR >> PinNumber) & 1;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    return (uint16_t) (pGPIOx->IDR);
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, bool Value) {
    if (Value == GPIO_PIN_SET) {
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, bool EnOrDi);

void GPIO_IRQHandling(uint8_t PinNumber);