/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Dec 3, 2025
 *      Author: Blah
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[] = {2,4,8,16,32,64,128,256,512};
uint16_t APB1_PreScaler[] = {2, 4, 8, 16};

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
