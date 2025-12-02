/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 17, 2025
 *      Author: Blah
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

// GPIO Pin numbers
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15


// GPIO modes
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFUN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

// GPIO Output types
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

// GPIO Speed modes
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MED 1
#define GPIO_SPEED_HIGH 2
#define GPIO_SPEED_VHIGH 3

// GPIO pin pull up AND pull down config macros
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

// Handle structure for GPIO pin

typedef struct {
	uint8_t GPIO_PinNumber; // Possible values defined in @GPIO_PIN_NO
	uint8_t GPIO_PinMode; // Possible values defined in @GPIO_PIN_MODE
	uint8_t GPIO_PinSpeed; // Possible values defined in @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl; // Possible values defined in @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType; // Possible values defined in @GPIO_PIN_OP
	uint8_t GPIO_PinAltFunMode; //
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx; // Holds base address of GPIO port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // Holds GPIO pin configuration settings
} GPIO_Handle_t;

// APIs supported by driver

// Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
// Clock control
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);
// Read
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
// Write
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
// IRQ Config and ISR Handling
void GPIO_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
