//
// Created by Oguz Kagan YAGLIOGLU on 9/22/2022.
// www.oguzkagan.xyz
//

#include "stm32f103xx.h"

void EXTI15_10_IRQHandler() {
    GPIO_IRQHandling(GPIO_PIN_15);
    GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
}

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
    gpioButton.GPIO_PinConfig.GPIO_PinIRQ_TRIG = GPIO_IRQ_RT;

    GPIO_PeriClockControl(GPIOC, true);
    GPIO_Init(&gpioLED);
    GPIO_Init(&gpioButton);

    GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);
    GPIO_IRQSetPriority(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY_15);


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (true) {

    }
#pragma clang diagnostic pop
    return 0;
}