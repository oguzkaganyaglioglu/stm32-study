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


    GPIO_PeriClockControl(GPIOC, true);
    GPIO_Init(&gpioLED);


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
        softDelay();
    }
#pragma clang diagnostic pop
    return 0;
}