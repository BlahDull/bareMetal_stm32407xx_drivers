/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Dec 3, 2025
 *      Author: Blah
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[] = {2,4,8,16,32,64,128,256,512};
uint16_t APB1_PreScaler[] = {2, 4, 8, 16};

void RCC_OscConfig(RCC_OscConfig_t* config) {
	if (config->RCC_OscType == RCC_OSCTYPE_HSE) {
		RCC->CR &= ~(1 << RCC_CR_HSEON_POS);
		if (config->RCC_HSEState == RCC_HSESTATE_BYPASS) {
			RCC->CR |= (1 << RCC_CR_HSEBYP_POS);
		}
		RCC->CR |= (1 << RCC_CR_HSEON_POS);
		while (!((RCC->CR >> (RCC_CR_HSERDY_POS - 1)) & 1));
	}
	if (config->PLL.PLL_State != PLL_NONE) {
		if (config->PLL.PLL_State == PLL_ON) {
			RCC->PLLCFGR &= ~(0b1111U << RCC_PLLCFGR_PLLQ_POS);
			RCC->PLLCFGR |= (config->PLL.PLL_DIV_Q << RCC_PLLCFGR_PLLQ_POS);
			RCC->PLLCFGR &= ~(0b11U << RCC_PLLCFGR_PLLP_POS);
			RCC->PLLCFGR |= (config->PLL.PLL_DIV_P << RCC_PLLCFGR_PLLP_POS);
			RCC->PLLCFGR &= ~(0b111111111U << RCC_PLLCFGR_PLLN_POS);
			RCC->PLLCFGR |= (config->PLL.PLL_MULT_N << RCC_PLLCFGR_PLLN_POS);
			RCC->PLLCFGR &= ~(0b111111U << RCC_PLLCFGR_PLLM_POS);
			RCC->PLLCFGR |= (config->PLL.PLL_DIV_M << RCC_PLLCFGR_PLLM_POS);
			RCC->PLLCFGR |= (config->PLL.PLL_Source << RCC_PLLCFGR_PLLSRC_POS);
			RCC->CR |= (1 << RCC_CR_PLLON_POS);
			while (!(RCC->CR & (1 << RCC_CR_PLLRDY_POS)));
		}
	}
}

void RCC_ClockConfig(RCC_ClockConfig_t* config) {
	if (config->RCC_ClockType & RCC_ClockType_HCLK) {
		RCC->CFGR |= (config->RCC_AHB_ClockDiv << RCC_CFGR_HPRE_POS);
	}
	FLASH_INTERFACE->FLASH_ACR &= ~(0b111U << FLASH_ACR_LATENCY_POS);
	FLASH_INTERFACE->FLASH_ACR |= (config->Flatency);
	while (((FLASH_INTERFACE->FLASH_ACR >> (FLASH_ACR_LATENCY_POS)) & (0b111U)) != config->Flatency);
	if (config->RCC_ClockType & RCC_ClockType_PCLK1) {
		RCC->CFGR |= (config->RCC_APB1_ClockDiv << RCC_CFGR_PPRE1_POS);
	}
	if (config->RCC_ClockType & RCC_ClockType_PCLK2) {
		RCC->CFGR |= (config->RCC_APB2_ClockDiv << RCC_CFGR_PPRE2_POS);
	}
	RCC->CFGR |= (config->RCC_SysClockSrc << RCC_CFGR_SW_POS);
	if (config->RCC_SysClockSrc != RCC_SysClockSrc_HSI) {
		RCC->CR &= ~(1 << RCC_CR_HSION_POS);
	}
}

void RCC_ConfigureMCO1(uint8_t clk_select, uint8_t prescaler) {
	RCC->CFGR |= (prescaler << RCC_CFGR_MCO1PRE_POS);
	RCC->CFGR |= (clk_select << RCC_CFGR_MCO1_POS);
}

uint32_t RCC_GetPLCK1Value() {
	uint32_t pCLK1 = 0, sysClk = 0;
	uint8_t clkSrc = 0, tmp = 0, AHBPrescaler = 0, APB1Prescaler = 0;
	clkSrc = ((RCC->CFGR >> 2) & 0x03);
	switch (clkSrc) {
	case 0:
		sysClk = 16000000;
		break;
	case 1:
		sysClk = 8000000;
		break;
	case 2:
		break;
	}
	tmp = ((RCC->CFGR >> 4) & 0xF);
	if (tmp < 8) {
		AHBPrescaler = 1;
	} else {
		AHBPrescaler = AHB_PreScaler[tmp - 8];
	}

	tmp = ((RCC->CFGR >> 10) & 0x7);
	if (tmp < 4) {
		APB1Prescaler = 1;
	} else {
		APB1Prescaler = APB1_PreScaler[tmp - 4];
	}
	pCLK1 = (sysClk / AHBPrescaler) / APB1Prescaler;
	return pCLK1;
}

uint32_t RCC_GetPLCK2Value() {
	uint32_t pCLK1 = 0, sysClk = 0;
	uint8_t clkSrc = 0, tmp = 0, AHBPrescaler = 0, APB1Prescaler = 0;
	clkSrc = ((RCC->CFGR >> 2) & 0x03);
	switch (clkSrc) {
	case 0:
		sysClk = 16000000;
		break;
	case 1:
		sysClk = 8000000;
		break;
	case 2:
		break;
	}
	tmp = ((RCC->CFGR >> 4) & 0xF);
	if (tmp < 8) {
		AHBPrescaler = 1;
	} else {
		AHBPrescaler = AHB_PreScaler[tmp - 8];
	}

	tmp = ((RCC->CFGR >> 13) & 0x7);
	if (tmp < 4) {
		APB1Prescaler = 1;
	} else {
		APB1Prescaler = APB1_PreScaler[tmp - 4];
	}
	pCLK1 = (sysClk / AHBPrescaler) / APB1Prescaler;
	return pCLK1;
}
