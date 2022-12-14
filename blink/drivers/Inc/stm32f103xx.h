//
// Created by Oguz Kagan YAGLIOGLU on 9/19/2022.
// www.oguzkagan.xyz
//

#ifndef STM32F1XX_DRIVERS_STM32F103XX_H
#define STM32F1XX_DRIVERS_STM32F103XX_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ----------------------------------------------------------------
 * Peripheral register definition structures
 * ----------------------------------------------------------------
 * */

/**
 * ARM Cortex-Mx Processor NVIC ISERx register addresses
 * */
#define NVIC_ISER_0                 ((volatile uint32_t *)0xE000E100)
#define NVIC_ISER_1                 ((volatile uint32_t *)0xE000E104)
#define NVIC_ISER_2                 ((volatile uint32_t *)0xE000E108)
#define NVIC_ISER_3                 ((volatile uint32_t *)0xE000E10C)
#define NVIC_ISER_4                 ((volatile uint32_t *)0xE000E110)
#define NVIC_ISER_5                 ((volatile uint32_t *)0xE000E114)
#define NVIC_ISER_6                 ((volatile uint32_t *)0xE000E118)
#define NVIC_ISER_7                 ((volatile uint32_t *)0xE000E11C)

/**
 * ARM Cortex-Mx Processor NVIC ICERx register addresses
 * */
#define NVIC_ICER_0                 ((volatile uint32_t *) 0XE000E180)
#define NVIC_ICER_1                 ((volatile uint32_t *) 0XE000E184)
#define NVIC_ICER_2                 ((volatile uint32_t *) 0XE000E188)
#define NVIC_ICER_3                 ((volatile uint32_t *) 0XE000E18C)
#define NVIC_ICER_4                 ((volatile uint32_t *) 0XE000E190)
#define NVIC_ICER_5                 ((volatile uint32_t *) 0XE000E194)
#define NVIC_ICER_6                 ((volatile uint32_t *) 0XE000E198)
#define NVIC_ICER_7                 ((volatile uint32_t *) 0XE000E19C)

/**
 * ARM Cortex-Mx Processor NVIC PR register addresses
 * */
#define NVIC_IPR_BASE_ADDR          ((volatile uint32_t *) 0xE000E400)

/**
 * Arm Cortex-Mx Processor number of priority bits implemented in Interrupt Priority Register
 * */
#define NO_IPR_BITS_IMPLEMENTED     4

/**
 * register definition structure for the NVIC ISER
 * */
typedef struct {
    volatile uint32_t REG_NUM[8];
} NVIC_ISER_RegDef_t;

/**
 * register definition structure for the NVIC ICER
 * */
typedef struct {
    volatile uint32_t REG_NUM[8];
} NVIC_ICER_RegDef_t;

/**
 * register definition for the NVIC ISER
 * */
#define NVIC_ISER  ((NVIC_ISER_RegDef_t *) NVIC_ISER_0)

/**
 * register definition for the NVIC ICER
 * */
#define NVIC_ICER  ((NVIC_ICER_RegDef_t *) NVIC_ICER_0)
/* ---------------------------------------------------------------- */

/**
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDR             0x08000000UL
#define SRAM1_BASE_ADDR             0x20000000UL
#define ROM_BASE_ADDR               0x1FFFF000UL
#define SRAM                        SRAM1_BASE_ADDR

/**
 * AHBx and APBx Bus Peripheral base addresses
 * */

#define PERIPHERAL_BASE_ADDR        0x40000000UL
#define APB1_BASE_ADDR              PERIPHERAL_BASE_ADDR
#define APB2_BASE_ADDR              0x40010000UL
#define AHB_BASE_ADDR               0x40018000UL
#define AHB1_BASE_ADDR              AHB_BASE_ADDR

#define AHB_ADD_OFFSET(OFFSET)      (AHB_BASE_ADDR + (OFFSET))
#define APB1_ADD_OFFSET(OFFSET)     (APB1_BASE_ADDR + (OFFSET))
#define APB2_ADD_OFFSET(OFFSET)     (APB2_BASE_ADDR + (OFFSET))

/**
 * Base addresses of peripherals which are hanging on AHB bus
 * */
#define SDIO_BASE_ADDR                       AHB_ADD_OFFSET(0x0000UL)
#define DMA1_BASE_ADDR                       AHB_ADD_OFFSET(0x8000UL)
#define DMA2_BASE_ADDR                       AHB_ADD_OFFSET(0x8400UL)
#define RCC_BASE_ADDR                        AHB_ADD_OFFSET(0x9000UL)
#define FLASH_MEMORY_INTERFACE_BASE_ADDR     AHB_ADD_OFFSET(0xA000UL)
#define CRC_BASE_ADDR                        AHB_ADD_OFFSET(0xB000UL)
#define ETHERNET_BASE_ADDR                   AHB_ADD_OFFSET(0x10000UL)
#define USB_OTG_FS_BASE_ADDR                 AHB_ADD_OFFSET(0xFFE8000UL)
#define FSMC_BASE_ADDR                       AHB_ADD_OFFSET(0x5FFE8000UL)

/**
 * Base addresses of peripherals which are hanging on APB1 bus
 * */

#define I2C1_BASE_ADDR                       APB1_ADD_OFFSET(0x5400UL)
#define I2C2_BASE_ADDR                       APB1_ADD_OFFSET(0x5800UL)

#define SPI2_BASE_ADDR                       APB1_ADD_OFFSET(0x3800UL)
//#define SPI3_BASE_ADDR                     APB1_ADD_OFFSET(0x3C00UL)

#define USART2_BASE_ADDR                     APB1_ADD_OFFSET(0x4400UL)
#define USART3_BASE_ADDR                     APB1_ADD_OFFSET(0x4800UL)

//#define UART4_BASE_ADDR                    APB1_ADD_OFFSET(0x4C00UL)
//#define UART5_BASE_ADDR                    APB1_ADD_OFFSET(0x5000UL)

#define TIM2_BASE_ADDR                       APB1_ADD_OFFSET(0x0000UL)
#define TIM3_BASE_ADDR                       APB1_ADD_OFFSET(0x0400UL)
#define TIM4_BASE_ADDR                       APB1_ADD_OFFSET(0x0800UL)
//#define TIM5_BASE_ADDR                     APB1_ADD_OFFSET(0x0C00UL)
//#define TIM6_BASE_ADDR                     APB1_ADD_OFFSET(0x1000UL)
//#define TIM7_BASE_ADDR                     APB1_ADD_OFFSET(0x1400UL)
//#define TIM12_BASE_ADDR                    APB1_ADD_OFFSET(0x1800UL)
//#define TIM13_BASE_ADDR                    APB1_ADD_OFFSET(0x1C00UL)
//#define TIM14_BASE_ADDR                    APB1_ADD_OFFSET(0x2000UL)

#define RTC_BASE_ADDR                        APB1_ADD_OFFSET(0x2800UL)

#define WWDG_BASE_ADDR                       APB1_ADD_OFFSET(0x2C00UL)
#define IWDG_BASE_ADDR                       APB1_ADD_OFFSET(0x3000UL)

#define PWR_BASE_ADDR                        APB1_ADD_OFFSET(0x7000UL)
#define DAC_BASE_ADDR                        APB1_ADD_OFFSET(0x7400UL)

/**
 * Base addresses of peripherals which are hanging on APB2 bus
 * */

#define GPIOA_BASE_ADDR                      APB2_ADD_OFFSET(0x0800UL)
#define GPIOB_BASE_ADDR                      APB2_ADD_OFFSET(0x0C00UL)
#define GPIOC_BASE_ADDR                      APB2_ADD_OFFSET(0x1000UL)
#define GPIOD_BASE_ADDR                      APB2_ADD_OFFSET(0x1400UL)
#define GPIOE_BASE_ADDR                      APB2_ADD_OFFSET(0x1800UL)
//#define GPIOF_BASE_ADDR                    APB2_ADD_OFFSET(0x1C00UL)
//#define GPIOG_BASE_ADDR                    APB2_ADD_OFFSET(0x2000UL)

#define SPI1_BASE_ADDR                       APB2_ADD_OFFSET(0x3000UL)

#define USART1_BASE_ADDR                     APB2_ADD_OFFSET(0x3800UL)

#define ADC1_BASE_ADDR                       APB2_ADD_OFFSET(0x2400UL)
#define ADC2_BASE_ADDR                       APB2_ADD_OFFSET(0x2800UL)
#define ADC3_BASE_ADDR                       APB2_ADD_OFFSET(0x3C00UL)

#define TIM1_BASE_ADDR                       APB2_ADD_OFFSET(0x2C00UL)
//#define TIM8_BASE_ADDR                     APB2_ADD_OFFSET(0x3400UL)
//#define TIM9_BASE_ADDR                     APB2_ADD_OFFSET(0x4C00UL)
//#define TIM10_BASE_ADDR                    APB2_ADD_OFFSET(0x5000UL)
//#define TIM11_BASE_ADDR                    APB2_ADD_OFFSET(0x5400UL)

#define EXTI_BASE_ADDR                       APB2_ADD_OFFSET(0x0400UL)

#define AFIO_BASE_ADDR                       APB2_ADD_OFFSET(0x0000UL)

/*
 * ----------------------------------------------------------------
 * Peripheral register definition structures
 * ----------------------------------------------------------------
 * */

/**
 * peripheral register definition structure for the GPIO
 * */
typedef struct {
    volatile uint32_t CR[2];     // Port configuration register         CR[0]: CRL (Address offset 0x00), CR[1]: CRH (Address offset 0x04),
    volatile uint32_t IDR;       // Port input data register            Address offset 0x08
    volatile uint32_t ODR;       // Port output data register           Address offset 0x0C
    volatile uint32_t BSRR;      // Port bit set/reset register         Address offset 0x10
    volatile uint32_t BRR;       // Port bit reset register             Address offset 0x14
    volatile uint32_t LCKR;      // Port bit set/reset register         Address offset 0x18
} GPIO_RegDef_t;

/**
 * peripheral register definition structure for the RCC
 * */
typedef struct {
    volatile uint32_t CR;       // Clock control register                   Address offset 0x00
    volatile uint32_t CFGR;     // Clock configuration register             Address offset 0x04
    volatile uint32_t CIR;      // Clock interrupt register                 Address offset 0x08
    volatile uint32_t APB2RSTR; // APB2 peripheral reset register           Address offset 0x0C
    volatile uint32_t APB1RSTR; // APB1 peripheral reset register           Address offset 0x10
    volatile uint32_t AHBENR;   // AHB Peripheral Clock enable register     Address offset 0x14
    volatile uint32_t APB2ENR;  // APB2 peripheral clock enable register    Address offset 0x18
    volatile uint32_t APB1ENR;  // APB1 peripheral clock enable register    Address offset 0x1C
    volatile uint32_t BDCR;     // Backup domain control register           Address offset 0x20
    volatile uint32_t CSR;      // Control/status register                  Address offset 0x24
    volatile uint32_t AHBSTR;   // AHB peripheral clock reset register      Address offset 0x28
    volatile uint32_t CFGR2;    // Clock configuration register2            Address offset 0x2C
} RCC_RegDef_t;

/**
 * peripheral register definition structure for the EXTI
 * */
typedef struct {
    volatile uint32_t IMR;       // Interrupt mask register                 Address offset 0x00
    volatile uint32_t EMR;       // Event mask register                     Address offset 0x04
    volatile uint32_t RTSR;      // Rising trigger selection register       Address offset 0x08
    volatile uint32_t FTSR;      // Falling trigger selection register      Address offset 0x0C
    volatile uint32_t SWIER;     // Software interrupt event register       Address offset 0x10
    volatile uint32_t PR;        // Pending register                        Address offset 0x14
} EXTI_RegDef_t;

/**
 * peripheral register definition structure for the AFIO
 * */
typedef struct {
    volatile uint32_t EVCR;      // Event control register                              Address offset 0x00
    volatile uint32_t MAPR;      // AF remap and debug I/O configuration register       Address offset 0x04
    volatile uint32_t EXTICR[4]; // External interrupt configuration register 1-4       Address offset 0x08 - 0x14
    volatile uint32_t MAPR2;     // AF remap and debug I/O configuration register2      Address offset 0x04
} AFIO_RegDef_t;

/*
 * Peripheral definitions
 * */

#define GPIOA ((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t *) GPIOE_BASE_ADDR)
//#define GPIOF ((GPIO_RegDef_t *) GPIOF_BASE_ADDR)
//#define GPIOG ((GPIO_RegDef_t *) GPIOG_BASE_ADDR)

#define RCC   ((RCC_RegDef_t *) RCC_BASE_ADDR)
#define EXTI  ((EXTI_RegDef_t *) EXTI_BASE_ADDR)
#define AFIO  ((AFIO_RegDef_t *) AFIO_BASE_ADDR)

/*
 * Clock enable/disable macros for AFIO peripheral
 * */
#define AFIO_PCLK_EN()       (RCC->APB2ENR |= (1 << 0))

#define AFIO_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 0))


/*
 * Clock enable/disable macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_EN()       (RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()       (RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()       (RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()       (RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()       (RCC->APB2ENR |= (1 << 6))

#define GPIOA_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()       (RCC->APB2ENR &= ~(1 << 6))


/*
 * GPIOx peripheral reset macros
 * */
#define GPIOA_REG_RST()       do{(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));} while(false)
#define GPIOB_REG_RST()       do{(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));} while(false)
#define GPIOC_REG_RST()       do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while(false)
#define GPIOD_REG_RST()       do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));} while(false)
#define GPIOE_REG_RST()       do{(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while(false)


/**
 * returns gpio port code for the given GPIO base address
 * */
#define GPIO_BASEADDR_TO_PORTCODE(x)   ((x == GPIOA) ? 0b0000 : \
                                        (x == GPIOB) ? 0b0001 : \
                                        (x == GPIOC) ? 0b0010 : \
                                        (x == GPIOD) ? 0b0011 : \
                                        (x == GPIOE) ? 0b0100 : 0)

/**
 * @IRQ_NUMBERS
 * IRQ(Interrupt Request) numbers
 * */
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40

/**
 * @IRQ_PRIORITES
 * IRQ(Interrupt Request) priorities
 * */
#define NVIC_IRQ_PRIORITY_0     0
#define NVIC_IRQ_PRIORITY_1     1
#define NVIC_IRQ_PRIORITY_2     2
#define NVIC_IRQ_PRIORITY_3     3
#define NVIC_IRQ_PRIORITY_4     4
#define NVIC_IRQ_PRIORITY_5     5
#define NVIC_IRQ_PRIORITY_6     6
#define NVIC_IRQ_PRIORITY_7     7
#define NVIC_IRQ_PRIORITY_8     8
#define NVIC_IRQ_PRIORITY_9     9
#define NVIC_IRQ_PRIORITY_10    10
#define NVIC_IRQ_PRIORITY_11    11
#define NVIC_IRQ_PRIORITY_12    12
#define NVIC_IRQ_PRIORITY_13    13
#define NVIC_IRQ_PRIORITY_14    14
#define NVIC_IRQ_PRIORITY_15    15

/*
 * Clock enable/disable macros for I2Cx peripherals
 * */
#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock enable/disable macros for SPIx peripherals
 * */
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))

/*
 * Clock enable/disable macros for USARTx peripherals
 * */
#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1 << 18))

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 18))

// some generic macros
#define ENABLE                      1
#define DISABLE                     0
#define SET                         ENABLE
#define RESET                       DISABLE
#define GPIO_PIN_SET                SET
#define GPIO_PIN_RESET              RESET

#define GPIO_OUT_INVERTED           1  // 1 => GPIO_PIN_SET: LOW, GPIO_PIN_RESET: HIGH ; 0 => GPIO_PIN_SET: HIGH, GPIO_PIN_RESET: LOW
#define GPIO_INVERT_IF_NEEDED(x)    (x^GPIO_OUT_INVERTED) // make sure GPIO_PIN_SET is HIGH, GPIO_PIN_RESET is LOW

#include "stm32f103xx_gpio_driver.h"

#endif //STM32F1XX_DRIVERS_STM32F103XX_H
