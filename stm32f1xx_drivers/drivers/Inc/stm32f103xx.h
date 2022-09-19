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

#define PERIPHERAL_BASE_ADDRESS     0x40000000UL
#define APB1_BASE_ADDRESS           PERIPHERAL_BASE_ADDRESS
#define APB2_BASE_ADDRESS           0x40010000UL
#define AHB_BASE_ADDRESS            0x40018000UL
#define AHB1_BASE_ADDRESS           AHB_BASE_ADDRESS

#endif //STM32F1XX_DRIVERS_STM32F103XX_H
