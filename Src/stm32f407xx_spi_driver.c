/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 20, 2025
 *      Author: Blah
 */

#include "stm32f407xx_spi_driver.h"

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnOrDi) {
	if (EnOrDi) {
		if (pSPIx == SPI1) SPI1_PCLK_EN();
		else if (pSPIx == SPI2) SPI2_PCLK_EN();
		else if (pSPIx == SPI3) SPI3_PCLK_EN();
		else if (pSPIx == SPI4) SPI4_PCLK_EN();
	} else {
		if (pSPIx == SPI1) SPI1_PCLK_DI();
		else if (pSPIx == SPI2) SPI2_PCLK_DI();
		else if (pSPIx == SPI3) SPI3_PCLK_DI();
		else if (pSPIx == SPI4) SPI4_PCLK_DI();
	}
}
void SPI_Init(SPI_Handle_t* pSPIHandle) {
	uint32_t tmp = 0;

	// Enable PCLK
	SPI_PeriClockControl(pSPIHandle->pSP, ENABLE);

	// config device mode
	tmp = pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_POS;
	pSPIHandle->pSP->SPI_CR1 |= (tmp);
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// bidi mode cleared
		tmp = 0;
		tmp = (1 << SPI_CR1_BIDIMODE_POS);
		pSPIHandle->pSP->SPI_CR1 &= ~(tmp);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// bidi mode set
		tmp = 0;
		tmp = (1 << SPI_CR1_BIDIMODE_POS);
		pSPIHandle->pSP->SPI_CR1 |= (tmp);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RXONLY) {
		// bidi mode set
		tmp = 0;
		tmp = (1 << SPI_CR1_BIDIMODE_POS);
		pSPIHandle->pSP->SPI_CR1 |= (tmp);
		// rxonly bit set
		tmp = 0;
		tmp = (1 << SPI_CR1_RXONLY_POS);
		pSPIHandle->pSP->SPI_CR1 |= (tmp);
	}
	// configure speed
	tmp = 0;
	tmp = (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_POS);
	pSPIHandle->pSP->SPI_CR1 |= (tmp);
	// configure DFF
	tmp = 0;
	tmp = (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF_POS);
	pSPIHandle->pSP->SPI_CR1 |= (tmp);
	// configure CPHA
	tmp = 0;
	tmp = (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_POS);
	pSPIHandle->pSP->SPI_CR1 |= (tmp);
	// configure CPOL
	tmp = 0;
	tmp = (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_POS);
	pSPIHandle->pSP->SPI_CR1 |= (tmp);
	// configure SSM
	tmp = 0;
	tmp = (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_POS);
	pSPIHandle->pSP->SPI_CR1 |= (tmp);
}
void SPI_DeInit(SPI_RegDef_t* pSPIHandle) {
	pSPIHandle->SPI_CR1 &= ~(1 << SPI_CR1_SPE_POS);
}


void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE_POS);
	} else {
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE_POS);
	}
}

void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI_POS);
	} else {
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI_POS);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE_POS);
	} else {
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE_POS);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName) {
	if (pSPIx->SPI_SR & flagName) return FLAG_SET;
	return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTXBuffer, uint32_t len) {
	while (len) {
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF_POS)) {
			// 16 bit
			pSPIx->SPI_DR = *((uint16_t*)pTXBuffer);
			len -= 2;
			(uint16_t*)pTXBuffer++;
		} else {
			// 8 bit
			pSPIx->SPI_DR = *pTXBuffer;
			len--;
			pTXBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRXBuffer, uint32_t len) {
	while (len) {
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF_POS)) {
			// 16 bit
			//pSPIx->SPI_DR = *((uint16_t*)pTXBuffer);
			*((uint16_t*) pRXBuffer) = pSPIx->SPI_DR;
			len -= 2;
			(uint16_t*)pRXBuffer++;
		} else {
			// 8 bit
			//pSPIx->SPI_DR = *pTXBuffer;
			*pRXBuffer = pSPIx->SPI_DR;
			len--;
			pRXBuffer++;
		}
	}
}
void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4, iprx_section = IRQNumber % 4;
	uint8_t shift_num = (8 * iprx_section) + (8 - NO_PR_BITS_IMPL);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_num);
}

uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pTXBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_TX) {
		pSPIHandle->pTxBuffer = pTXBuffer;
		pSPIHandle->TxLen = len;
		pSPIHandle->TxState = SPI_BUSY_TX;

		pSPIHandle->pSP->SPI_CR2 |= (1 << SPI_CR2_TXEIE_POS);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pRXBuffer, uint32_t len) {
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_RX) {
		pSPIHandle->pRxBuffer = pRXBuffer;
		pSPIHandle->RxLen = len;
		pSPIHandle->RxState = SPI_BUSY_RX;

		pSPIHandle->pSP->SPI_CR2 |= (1 << SPI_CR2_RXNEIE_POS);
	}
	return state;
}

__weak__ void SPI_ApplicationEventCallback (SPI_Handle_t* pSPIHandle, uint8_t SPI_EVENT) {

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle) {
	if (pHandle->pSP->SPI_CR1 & (1 << SPI_CR1_DFF_POS)) {
		// 16 bit
		pHandle->pSP->SPI_DR = *((uint16_t*)pHandle->pTxBuffer);
		pHandle->TxLen -= 2;
		(uint16_t*)pHandle->pTxBuffer++;
	} else {
		// 8 bit
		pHandle->pSP->SPI_DR = *pHandle->pTxBuffer;
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}
	if (pHandle->TxLen == 0) {
		SPI_CloseTransmission(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle) {
	if (pHandle->pSP->SPI_CR1 & (1 << SPI_CR1_DFF_POS)) {
		// 16 bit
		*((uint16_t*)pHandle->pRxBuffer) = (uint16_t)pHandle->pSP->SPI_DR;
		pHandle->RxLen -= 2;
		(uint16_t*)pHandle->pRxBuffer++;
	} else {
		// 8 bit
		*(pHandle->pRxBuffer) = (uint8_t)pHandle->pSP->SPI_DR;
		pHandle->RxLen--;
		pHandle->pRxBuffer++;
	}
	if (pHandle->RxLen == 0) {
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle) {
	uint8_t tmp;
	if (pHandle->TxState != SPI_BUSY_TX) {
		tmp = pHandle->pSP->SPI_DR;
		tmp = pHandle->pSP->SPI_SR;
	}
	(void) tmp;
	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);
}


void SPI_IRQHandling(SPI_Handle_t *pHandle) {
	uint8_t tmp1, tmp2;
	tmp1 = pHandle->pSP->SPI_SR & (1 << SPI_SR_TXE_POS);
	tmp2 = pHandle->pSP->SPI_CR2 & (1 << SPI_CR2_TXEIE_POS);

	if (tmp1 && tmp2) {
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	tmp1 = pHandle->pSP->SPI_SR & (1 << SPI_SR_RXNE_POS);
	tmp2 = pHandle->pSP->SPI_CR2 & (1 << SPI_CR2_RXNEIE_POS);
	if (tmp1 && tmp2) {
		// handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	tmp1 = pHandle->pSP->SPI_SR & (1 << SPI_SR_OVR_POS);
	tmp2 = pHandle->pSP->SPI_CR2 & (1 << SPI_CR2_ERRIE_POS);
	if (tmp1 && tmp2) {
		// handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}

}

void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx) {
	uint8_t tmp;
	tmp = pSPIx->SPI_DR;
	tmp = pSPIx->SPI_SR;
	(void) tmp;
}
void SPI_CloseTransmission(SPI_Handle_t* pHandle) {
	pHandle->pSP->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE_POS);
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
	//SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
}
void SPI_CloseReception(SPI_Handle_t* pHandle) {
	pHandle->pSP->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE_POS);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
	//SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
}
