/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Dec 3, 2025
 *      Author: Blah
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t PLL_State;
	uint8_t PLL_Source;
	uint8_t PLL_DIV_M;
	uint16_t PLL_MULT_N;
	uint8_t PLL_DIV_P;
	uint8_t PLL_DIV_Q;
} RCC_PLLConfig_t;

typedef struct {
	uint8_t RCC_OscType;
	uint8_t RCC_HSEState;
	uint8_t RCC_HSIState;
	RCC_PLLConfig_t PLL;
} RCC_OscConfig_t;

typedef struct {
	uint8_t RCC_ClockType;
	uint8_t RCC_SysClockSrc;
	uint32_t RCC_AHB_ClockDiv;
	uint32_t RCC_APB1_ClockDiv;
	uint32_t RCC_APB2_ClockDiv;
	uint8_t Flatency;
} RCC_ClockConfig_t;

#define RCC_OSCTYPE_NONE 0
#define RCC_OSCTYPE_HSE 1
#define RCC_OSCTYPE_HSI 2

#define RCC_HSESTATE_ON 0
#define RCC_HSESTATE_OFF 1
#define RCC_HSESTATE_BYPASS 2

#define RCC_HSISTATE_ON 0
#define RCC_HSISTATE_OFF 1

#define PLL_NONE 0
#define PLL_ON 1
#define PLL_OFF 2

#define PLL_SOURCE_HSI 0
#define PLL_SOURCE_HSE 1

#define RCC_ClockType_SYSCLK 0b0001U
#define RCC_ClockType_HCLK   0b0010U
#define RCC_ClockType_PCLK1  0b0100U
#define RCC_ClockType_PCLK2  0b1000U

#define RCC_SysClockSrc_HSI 0b00U
#define RCC_SysClockSrc_HSE 0b01U
#define RCC_SysClockSrc_PLL 0b10U

#define PLL_P_2 0b00U
#define PLL_P_4 0b01U
#define PLL_P_6 0b10U
#define PLL_P_8 0b11U

#define RCC_HPRE_NONE 0b0000U
#define RCC_HPRE_2 0b1000U
#define RCC_HPRE_4 0b1001U
#define RCC_HPRE_8 0b1010U
#define RCC_HPRE_16 0b1011U
#define RCC_HPRE_64 0b1100U
#define RCC_HPRE_128 0b1101U
#define RCC_HPRE_256 0b1110U
#define RCC_HPRE_512 0b1111U

#define RCC_PPRE1_NONE 0b000U
#define RCC_PPRE1_2 0b100U
#define RCC_PPRE1_4 0b101U
#define RCC_PPRE1_8 0b110U
#define RCC_PPRE1_16 0b111U

#define RCC_PPRE2_NONE 0b000U
#define RCC_PPRE2_2 0b100U
#define RCC_PPRE2_4 0b101U
#define RCC_PPRE2_8 0b110U
#define RCC_PPRE2_16 0b111U

#define FLatency_0WS 0b000U
#define FLatency_1WS 0b001U
#define FLatency_2WS 0b010U
#define FLatency_3WS 0b011U
#define FLatency_4WS 0b100U
#define FLatency_5WS 0b101U
#define FLatency_6WS 0b110U
#define FLatency_7WS 0b111U

#define RCC_MCO1_CLKSRC_HSI 0b00U
#define RCC_MCO1_CLKSRC_LSE 0b01U
#define RCC_MCO1_CLKSRC_HSE 0b10U
#define RCC_MCO1_CLKSRC_PLL 0b11U

#define RCC_MCO1_PRESCALER_NONE 0b000U
#define RCC_MCO1_PRESCALER_2 0b100U
#define RCC_MCO1_PRESCALER_3 0b101U
#define RCC_MCO1_PRESCALER_4 0b110U
#define RCC_MCO1_PRESCALER_5 0b111U

void RCC_OscConfig(RCC_OscConfig_t*);
void RCC_ClockConfig(RCC_ClockConfig_t*);
void RCC_ConfigureMCO1(uint8_t, uint8_t);
uint32_t RCC_GetPLCK1Value();
uint32_t RCC_GetPLCK2Value();

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
