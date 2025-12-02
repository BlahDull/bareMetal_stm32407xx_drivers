/*
 * stm32f407xx.gpio_driver.c
 *
 *  Created on: Nov 17, 2025
 *      Author: Blah
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t tmp = 0;

	// Enable PCLK
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// Configure the mode of gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= tmp;
	} else {
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// configure GPIO port selection in SYSCFG_EXTICR
		uint8_t tmp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4, tmp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFGEN_PCLK_EN();
		switch (tmp1) {
		case 0:
			SYSCFG->EXTICR1 = portCode << (tmp2 * 4);
			break;
		case 1:
			SYSCFG->EXTICR2 = portCode << (tmp2 * 4);
			break;
		case 2:
			SYSCFG->EXTICR3 = portCode << (tmp2 * 4);
			break;
		case 3:
			SYSCFG->EXTICR4 = portCode << (tmp2 * 4);
			break;
		}
		// enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	// configure speed
	tmp = 0;
	tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= tmp;
	// configure pupd
	tmp = 0;
	tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= tmp;
	// configure optype
	tmp = 0;
	tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= tmp;
	// configure alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN) {
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 7) {
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) % 8)));
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) % 8)));
		} else {
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
		}
	}
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE) GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF) GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG) GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH) GPIOH_REG_RESET();
	else if (pGPIOx == GPIOI) GPIOI_REG_RESET();
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {
	if (EnOrDi) {
		if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH) GPIOH_PCLK_EN();
		else if (pGPIOx == GPIOI) GPIOI_PCLK_EN();
	} else {
		if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH) GPIOH_PCLK_DI();
		else if (pGPIOx == GPIOI) GPIOI_PCLK_DI();
	}
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
	if (EnOrDi) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 63 && IRQNumber <= 95) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
				*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63) {
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber > 63 && IRQNumber <= 95) {
				*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4, iprx_section = IRQNumber % 4;
	uint8_t shift_num = (8 * iprx_section) + (8 - NO_PR_BITS_IMPL);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_num);
}

/*
 * @fn GPIO_PeriClockControl
 *
 * @brief - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param - base address of GPIO peripheral
 * @param - Enable or Disable macro
 *
 * @return- none
 *
 * @Note - none
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	// clear EXTI PR register
	if (EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}
}
