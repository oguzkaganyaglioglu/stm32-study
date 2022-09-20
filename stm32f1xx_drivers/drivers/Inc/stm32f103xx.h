//
// Created by Oguz Kagan YAGLIOGLU on 9/19/2022.
// www.oguzkagan.xyz
//

#ifndef STM32F1XX_DRIVERS_STM32F103XX_H
#define STM32F1XX_DRIVERS_STM32F103XX_H

/**
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDRESS          0x08000000UL
#define SRAM1_BASE_ADDRESS          0x20000000UL
#define ROM_BASE_ADDRESS            0x1FFFF000UL
#define SRAM                        SRAM1_BASE_ADDRESS

/**
 * AHBx and APBx Bus Peripheral base addresses
 * */

#define PERIPHERAL_BASE_ADDRESS     0x40000000UL
#define APB1_BASE_ADDRESS           PERIPHERAL_BASE_ADDRESS
#define APB2_BASE_ADDRESS           0x40010000UL
#define AHB_BASE_ADDRESS            0x40018000UL
#define AHB1_BASE_ADDRESS           AHB_BASE_ADDRESS

#define APB1_ADD_OFFSET(OFFSET) (APB1_BASE_ADDRESS + (OFFSET))
#define APB2_ADD_OFFSET(OFFSET) (APB2_BASE_ADDRESS + (OFFSET))

/**
 * Base addresses of peripherals which are hanging on APB1 bus
 * */

#define I2C1_BASE_ADDRESS           APB1_ADD_OFFSET(0x5400UL)
#define I2C2_BASE_ADDRESS           APB1_ADD_OFFSET(0x5800UL)

#define SPI2_BASE_ADDRESS           APB1_ADD_OFFSET(0x3800UL)
#define SPI3_BASE_ADDRESS           APB1_ADD_OFFSET(0x3C00UL)

#define USART2_BASE_ADDRESS         APB1_ADD_OFFSET(0x4400UL)
#define USART3_BASE_ADDRESS         APB1_ADD_OFFSET(0x4800UL)

#define UART4_BASE_ADDRESS          APB1_ADD_OFFSET(0x4C00UL)
#define UART5_BASE_ADDRESS          APB1_ADD_OFFSET(0x5000UL)

#define TIM2_BASE_ADDRESS           APB1_ADD_OFFSET(0x0000UL)
#define TIM3_BASE_ADDRESS           APB1_ADD_OFFSET(0x0400UL)
#define TIM4_BASE_ADDRESS           APB1_ADD_OFFSET(0x0800UL)
#define TIM5_BASE_ADDRESS           APB1_ADD_OFFSET(0x0C00UL)
#define TIM6_BASE_ADDRESS           APB1_ADD_OFFSET(0x1000UL)
#define TIM7_BASE_ADDRESS           APB1_ADD_OFFSET(0x1400UL)
#define TIM12_BASE_ADDRESS          APB1_ADD_OFFSET(0x1800UL)
#define TIM13_BASE_ADDRESS          APB1_ADD_OFFSET(0x1C00UL)
#define TIM14_BASE_ADDRESS          APB1_ADD_OFFSET(0x2000UL)

#define RTC_BASE_ADDRESS            APB1_ADD_OFFSET(0x2800UL)

#define WWDG_BASE_ADDRESS           APB1_ADD_OFFSET(0x2C00UL)
#define IWDG_BASE_ADDRESS           APB1_ADD_OFFSET(0x3000UL)

#define PWR_BASE_ADDRESS            APB1_ADD_OFFSET(0x7000UL)
#define DAC_BASE_ADDRESS            APB1_ADD_OFFSET(0x7400UL)

/**
 * Base addresses of peripherals which are hanging on APB2 bus
 * */

#define GPIOA_BASE_ADDRESS          APB2_ADD_OFFSET(0x0800UL)
#define GPIOB_BASE_ADDRESS          APB2_ADD_OFFSET(0x0C00UL)
#define GPIOC_BASE_ADDRESS          APB2_ADD_OFFSET(0x1000UL)
#define GPIOD_BASE_ADDRESS          APB2_ADD_OFFSET(0x1400UL)
#define GPIOE_BASE_ADDRESS          APB2_ADD_OFFSET(0x1800UL)
#define GPIOF_BASE_ADDRESS          APB2_ADD_OFFSET(0x1C00UL)
#define GPIOG_BASE_ADDRESS          APB2_ADD_OFFSET(0x2000UL)

#define SPI1_BASE_ADDRESS           APB2_ADD_OFFSET(0x3000UL)

#define USART1_BASE_ADDRESS         APB2_ADD_OFFSET(0x3800UL)

#define ADC1_BASE_ADDRESS           APB2_ADD_OFFSET(0x2400UL)
#define ADC2_BASE_ADDRESS           APB2_ADD_OFFSET(0x2800UL)
#define ADC3_BASE_ADDRESS           APB2_ADD_OFFSET(0x3C00UL)

#define TIM1_BASE_ADDRESS           APB2_ADD_OFFSET(0x2C00UL)
#define TIM8_BASE_ADDRESS           APB2_ADD_OFFSET(0x3400UL)
#define TIM9_BASE_ADDRESS           APB2_ADD_OFFSET(0x4C00UL)
#define TIM10_BASE_ADDRESS          APB2_ADD_OFFSET(0x5000UL)
#define TIM11_BASE_ADDRESS          APB2_ADD_OFFSET(0x5400UL)

#define EXTI_BASE_ADDRESS           APB2_ADD_OFFSET(0x0400UL)

#define AFIO_BASE_ADDRESS           APB2_ADD_OFFSET(0x0000UL)

#endif //STM32F1XX_DRIVERS_STM32F103XX_H
