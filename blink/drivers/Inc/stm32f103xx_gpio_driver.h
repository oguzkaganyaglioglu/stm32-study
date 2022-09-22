//
// Created by Oguz Kagan YAGLIOGLU on 9/20/2022.
// www.oguzkagan.xyz
//

#ifndef STM32F1XX_DRIVERS_STM32F103XX_GPIO_DRIVER_H
#define STM32F1XX_DRIVERS_STM32F103XX_GPIO_DRIVER_H

#include "stm32f103xx.h"


/**
 * @GPIO_PIN_MODES
 * */
#define GPIO_MODE_OUT_MAX_SPEED_2MHZ     0b10
#define GPIO_MODE_OUT_MAX_SPEED_10MHZ    0b01
#define GPIO_MODE_OUT_MAX_SPEED_50MHZ    0b11

#define GPIO_MODE_IN                        0b00
#define GPIO_MODE_OUT                       GPIO_MODE_OUT_MAX_SPEED_10MHZ

//#define GPIO_MODE_IT_FT                     0b100 // Interrupt trigger falling edge
//#define GPIO_MODE_IT_RT                     0b101 // Interrupt trigger rising edge
//#define GPIO_MODE_IT_RFT                    0b110 // Interrupt trigger falling and rising edge

/**
 * @GPIO_PIN_CNF
 * */
#define GPIO_CNF_IN_A                       0b00 // Analog mode
#define GPIO_CNF_IN_F                       0b01 // Floating input (reset state)
#define GPIO_CNF_IN_PUPD                    0b10 // Input with pull-up / pull-down
#define GPIO_CNF_IN_RESERVED                0b11 // RESERVED

#define GPIO_CNF_OUT_PP                     0b00 // General purpose output push-pull
#define GPIO_CNF_OUT_OD                     0b01 // General purpose output Open-drain
#define GPIO_CNF_OUT_AF_PP                  0b10 // Alternate function output Push-pull
#define GPIO_CNF_OUT_AF_OD                  0b11 // Alternate function output Open-drain

#define GPIO_PU                             1 // Pull-up
#define GPIO_PD                             0 // Pull-down

/**
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 * */
#define GPIO_PIN_0                          0
#define GPIO_PIN_1                          1
#define GPIO_PIN_2                          2
#define GPIO_PIN_3                          3
#define GPIO_PIN_4                          4
#define GPIO_PIN_5                          5
#define GPIO_PIN_6                          6
#define GPIO_PIN_7                          7
#define GPIO_PIN_8                          8
#define GPIO_PIN_9                          9
#define GPIO_PIN_10                         10
#define GPIO_PIN_11                         11
#define GPIO_PIN_12                         12
#define GPIO_PIN_13                         13
#define GPIO_PIN_14                         14
#define GPIO_PIN_15                         15

typedef struct {
    uint8_t GPIO_PinNumber;                 /** possible values from @GPIO_PIN_NUMBERS */
    uint8_t GPIO_PinMode;                   /** possible values values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinCNF;                    /** possible values values from @GPIO_PIN_CNF */
    bool GPIO_PinPuPdControl;
//    uint8_t GPIO_PinAltFuncMode;
} GPIO_PinConfig_t;

typedef struct {
    GPIO_RegDef_t *pGPIOx;                  //This pointer holds the base address of the GPIO port
    GPIO_PinConfig_t GPIO_PinConfig;        //This pointer holds the GPIO pin configuration settings
} GPIO_Handle_t;


/**
 * @fn          GPIO_Init
 *
 * @brief       This function initializes the GPIO peripheral for the given port
 *
 * @param[in]   pGPIOHandle
 *
 * @return      none
 *
 * @note        none
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @fn          GPIO_DeInit
 *
 * @brief       This function deinitializes the GPIO peripheral
 *
 * @param[in]   GPIO_RegDef_t base address of the GPIO peripheral for the given port
 *
 * @return      none
 *
 * @note        none
 * */
void GPIO_DeInit(GPIO_RegDef_t *GPIOx);


/**
 * @fn          GPIO_PeriClockControl
 *
 * @brief       This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]   pGPIOx   base address of the GPIO peripheral
 * @param[in]   EnOrDi   ENABLE or DISABLE macros
 *
 * @return      none
 *
 * @note        none
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, bool EnOrDi);

/**
 * @fn          GPIO_ReadFromInputPin
 *
 * @brief       This function reads the value from a specified GPIO pin
 *
 * @param[in]   pGPIOx      base address of the GPIO peripheral
 * @param[in]   PinNumber   pin number, possible values from @GPIO_PIN_NUMBERS
 *
 * @return      GPIO pin value: true (HIGH) or false (LOW)
 *
 * @note        none
 * */
bool GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @fn          GPIO_ReadFromInputPort
 *
 * @brief       This function reads the value from a specified GPIO port
 *
 * @param[in]   pGPIOx      base address of the GPIO peripheral
 *
 * @return      GPIO port value: uint16_t
 *
 * @note        none
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @fn          GPIO_WriteToOutputPin
 *
 * @brief       This function writes a value to the specified GPIO pin
 *
 * @param[in]   pGPIOx      base address of the GPIO peripheral
 * @param[in]   PinNumber   pin number, possible values from @GPIO_PIN_NUMBERS
 * @param[in]   Value       true: HIGH, false: LOW
 *
 * @return      none
 *
 * @note        if the pin is configured as INPUT; value(true) enable pull-up resistor, value(false) enable pull-down resistor
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, bool Value);

/**
 * @fn          GPIO_WriteToOutputPort
 *
 * @brief       This function writes the value to the specified GPIO port
 *
 * @param[in]   pGPIOx      base address of the GPIO peripheral
 * @param[in]   Value       value to be written to the GPIO port. 1:HIGH, 0:LOW
 *
 * @return      none
 *
 * @note        if a pin is configured as INPUT; 1 enable pull-up resistor, 0 enable pull-down resistor
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/**
 * @fn          GPIO_ToggleOutputPin
 *
 * @brief       This function toggles the specified GPIO pin
 *
 * @param[in]   pGPIOx      base address of the GPIO peripheral
 * @param[in]   PinNumber   pin number, possible values from @GPIO_PIN_NUMBERS
 *
 * @return      none
 *
 * @note        none
 * */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, bool EnOrDi);

void GPIO_IRQHandling(uint8_t PinNumber);


#endif //STM32F1XX_DRIVERS_STM32F103XX_GPIO_DRIVER_H
