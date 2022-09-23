//
// Created by Oguz Kagan YAGLIOGLU on 9/22/2022.
// www.oguzkagan.xyz
//

#include "stm32f103xx.h"


void softDelay() {
    for (uint32_t i = 0; i < 500000; ++i) asm("nop");
}

int main() {

    GPIO_Handle_t gpioLED;
    gpioLED.pGPIOx = GPIOC;
    gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLED.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_OUT_PP;

    GPIO_Handle_t gpioButton;
    gpioButton.pGPIOx = GPIOC;
    gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
    gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpioButton.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_IN_PUPD;
    gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;

    GPIO_PeriClockControl(GPIOC, true);
    GPIO_Init(&gpioLED);
    GPIO_Init(&gpioButton);


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, GPIO_INVERT_IF_NEEDED(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_15)));
    }
#pragma clang diagnostic pop
    return 0;
}