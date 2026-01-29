/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Blah
 */

#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

void USART_Init(USART_Handle_t *pUSARTHandle) {
	uint32_t tmp = 0;
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
		tmp |= (1 << USART_CR1_RE_POS);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
		tmp |= (1 << USART_CR1_TE_POS);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
		tmp |= (1 << USART_CR1_RE_POS);
		tmp |= (1 << USART_CR1_TE_POS);
	}

	tmp |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M_POS);

	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
		tmp |= (1 << USART_CR1_PCE_POS);
		tmp |= (0 << USART_CR1_PS_POS);
	} else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
		tmp |= (1 << USART_CR1_PCE_POS);
		tmp |= (1 << USART_CR1_PS_POS);
	}

	pUSARTHandle->pUSARTx->USART_CR1 = tmp;


	tmp = 0;
	tmp |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP_POS);
	pUSARTHandle->pUSARTx->USART_CR2 = tmp;


	tmp = 0;
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		tmp |= (1 << USART_CR3_CTSE_POS);
	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		tmp |= (1 << USART_CR3_RTSE_POS);
	} else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		tmp |= (1 << USART_CR3_RTSE_POS);
		tmp |= (1 << USART_CR3_CTSE_POS);
	}

	pUSARTHandle->pUSARTx->USART_CR3 = tmp;

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}
void USART_DeInit(USART_RegDef_t *pUSARTx) {

}

void USART_PeriClockControl(USART_RegDef_t* pUSARTx, uint8_t EnOrDi) {
	if (EnOrDi) {
		if (pUSARTx == USART1) USART1_PCLK_EN();
		else if (pUSARTx == USART2) USART2_PCLK_EN();
		else if (pUSARTx == USART3) USART3_PCLK_EN();
		else if (pUSARTx == USART6) USART6_PCLK_EN();
	} else {
		if (pUSARTx == USART1) USART1_PCLK_DI();
		else if (pUSARTx == USART2) USART2_PCLK_DI();
		else if (pUSARTx == USART3) USART3_PCLK_DI();
		else if (pUSARTx == USART6) USART6_PCLK_DI();
	}
}

void USART_PeripheralControl(USART_RegDef_t* pUSARTx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pUSARTx->USART_CR1 |= (1 << USART_CR1_UE_POS);
	} else {
		pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE_POS);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint16_t StatusFlagName) {
	if (pUSARTx->USART_SR & StatusFlagName) return FLAG_SET;
	return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t* pUSARTx, uint16_t StatusFlagName) {
	pUSARTx->USART_SR &= ~(1 << StatusFlagName);
}

void USART_SetBaudRate(USART_RegDef_t* pUSARTx, uint32_t BaudRate) {
	uint32_t PCLKx, usart_div, mantissa, fraction, tmp = 0;
	if (pUSARTx == USART1 || pUSARTx == USART6) {
		PCLKx = RCC_GetPLCK2Value();
	} else {
		PCLKx = RCC_GetPLCK2Value();
	}

	if (pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8_POS)) {
		usart_div = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		usart_div = ((25 * PCLKx) / (4 * BaudRate));
	}

	mantissa = usart_div / 100;
	tmp |= (mantissa << 4);
	fraction = (usart_div - (mantissa * 100));

	if (pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8_POS)) {
		fraction = (((fraction *  8) + 50) / 100) & ((uint8_t) 0x07);
	} else {
		fraction = (((fraction *  16) + 50) / 100) & ((uint8_t) 0x0F);
	}
	tmp |= fraction;

	pUSARTx->USART_BRR = tmp;
}

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len) {
	uint16_t *pdata;
	for (uint32_t i = 0; i < Len; i++) {
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t)0x1FF);
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				pTxBuffer += 2;
			} else {
				pTxBuffer++;
			}
		} else {
			pUSARTHandle->pUSARTx->USART_DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
	for (uint32_t i = 0; i < Len; i++) {
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR  & (uint16_t)0x1FF);
				pRxBuffer += 2;
			} else {
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		} else {
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				*pRxBuffer = pUSARTHandle->pUSARTx->USART_DR;
			} else {
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {
	uint8_t txState = pUSARTHandle->USART_TXState;
	if (txState != USART_BUSY_IN_TX) {
		pUSARTHandle->USART_TXLen = Len;
		pUSARTHandle->pTXBuffer = pTxBuffer;
		pUSARTHandle->USART_TXState = USART_BUSY_IN_TX;
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TXEIE_POS);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TCIE_POS);
	}
	return txState;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t rxState = pUSARTHandle->USART_RXState;
	if (rxState != USART_BUSY_IN_RX) {
		pUSARTHandle->USART_RXLen = Len;
		pUSARTHandle->pRXBuffer = pRxBuffer;
		pUSARTHandle->USART_RXState = USART_BUSY_IN_RX;
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RXNEIE_POS);
	}
	return rxState;
}

void USART_SendDataDMA(USART_Handle_t *pUSARTHandle) {
	pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_DMAT_POS);
}

void USART_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4, iprx_section = IRQNumber % 4;
	uint8_t shift_num = (8 * iprx_section) + (8 - NO_PR_BITS_IMPL);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_num);
}

static void USART_ClearIdleBit(USART_RegDef_t* pUSARTx) {
	uint8_t dummyread;
	dummyread = pUSARTx->USART_DR;
	(void) dummyread;

}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {
	uint32_t tmp1, tmp2, tmp3;
	tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_TC_POS);
	tmp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TCIE_POS);
	uint16_t* pdata;
	if (tmp1 && tmp2) {
		if (pUSARTHandle->USART_TXState == USART_BUSY_IN_TX) {
			if (!pUSARTHandle->USART_TXLen) {
				pUSARTHandle->pUSARTx->USART_SR &= ~(1 << USART_SR_TC_POS);
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TCIE_POS);
				pUSARTHandle->USART_TXState = USART_READY;
				pUSARTHandle->pTXBuffer = NULL;
				pUSARTHandle->USART_TXLen = 0;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_TXE_POS);
	tmp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TXEIE_POS);

	if (tmp1 && tmp2) {
		if (pUSARTHandle->USART_TXState == USART_BUSY_IN_TX) {
			if (pUSARTHandle->USART_TXLen > 0) {
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					pdata = (uint16_t*) pUSARTHandle->pTXBuffer;
					pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t) 0x1FF);
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						pUSARTHandle->pTXBuffer += 2;
						pUSARTHandle->USART_TXLen--;
					} else {
						pUSARTHandle->pTXBuffer++;
						pUSARTHandle->USART_TXLen--;
					}
				} else {
					pUSARTHandle->pUSARTx->USART_DR = (*pUSARTHandle->pTXBuffer & (uint8_t)0xFF);
					pUSARTHandle->pTXBuffer++;
					pUSARTHandle->USART_TXLen--;
				}
			}
			if (pUSARTHandle->USART_TXLen == 0) {
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TXEIE_POS);
			}
		}
	}

	tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_RXNE_POS);
	tmp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE_POS);

	if (tmp1 && tmp2) {
		if (pUSARTHandle->USART_RXState == USART_BUSY_IN_RX) {
			if (pUSARTHandle->USART_RXLen > 0) {
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						*((uint16_t*) pUSARTHandle->pRXBuffer) = (pUSARTHandle->pUSARTx->USART_DR & (uint16_t) 0x1FF);
						pUSARTHandle->pRXBuffer += 2;
						pUSARTHandle->USART_RXLen--;
					} else {
						*pUSARTHandle->pRXBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
						pUSARTHandle->pRXBuffer++;
						pUSARTHandle->USART_RXLen--;
					}
				} else {
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						*pUSARTHandle->pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
					} else {
						*pUSARTHandle->pRXBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
					}
					pUSARTHandle->pRXBuffer++;
					pUSARTHandle->USART_RXLen--;
				}
			}
			if (pUSARTHandle->USART_RXLen == 0) {
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_RXNEIE_POS);
				pUSARTHandle->USART_RXState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	tmp1 = pUSARTHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_CTSE_POS);
	tmp2 = pUSARTHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_CTSIE_POS);
	tmp3 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_CTS_POS);

	if (tmp1 && tmp2 && tmp3) {
		pUSARTHandle->pUSARTx->USART_SR &= ~(1 << USART_SR_CTS_POS);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}


	tmp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_IDLEIE_POS);
	tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_IDLE_POS);
	if (tmp1 && tmp2) {
		USART_ClearIdleBit(pUSARTHandle->pUSARTx);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}


	tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_ORE_POS);
	tmp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE_POS);

	if (tmp1 && tmp2) {
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ORE);
	}


	tmp1 = pUSARTHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_EIE_POS);
	if (tmp1) {
		tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_FE_POS);
		if (tmp1) USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_FE);
		tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_NF_POS);
		if (tmp1) USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_NF);
		tmp1 = pUSARTHandle->pUSARTx->USART_SR & (1 << USART_SR_ORE_POS);
		if (tmp1) USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_ORE);
	}

}

__weak__ void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv) {

}

