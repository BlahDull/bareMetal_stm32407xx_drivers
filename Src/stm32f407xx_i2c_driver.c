/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Nov 26, 2025
 *      Author: Blah
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"




void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi) {
		if (pI2Cx == I2C1) I2C1_PCLK_EN();
		else if (pI2Cx == I2C2) I2C2_PCLK_EN();
		else if (pI2Cx == I2C3) I2C3_PCLK_EN();
	} else {
		if (pI2Cx == I2C1) I2C1_PCLK_DI();
		else if (pI2Cx == I2C2) I2C2_PCLK_DI();
		else if (pI2Cx == I2C3) I2C3_PCLK_DI();
	}
}

void I2C_Init(I2C_Handle_t* pI2CHandle) {
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	uint32_t tmp = 0;
	// configure FREQ bits
	tmp = 0;
	tmp |= RCC_GetPLCK1Value() / (1000000U);
	pI2CHandle->pI2Cx->I2C_CR2 = (tmp & 0x3F);
	// configure slave address
	tmp = 0;
	tmp |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tmp |= (1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tmp;
	// ccr calculation
	uint16_t ccr_value = 0;
	tmp = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// standard mode
		ccr_value = (RCC_GetPLCK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tmp |= (ccr_value & 0xFFF);
	} else {
		// fast mode
		tmp |= (1 << 15);
		tmp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY_POS);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPLCK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccr_value = (RCC_GetPLCK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tmp |= (ccr_value & 0xFFF);
	}
	// configure ccr
	pI2CHandle->pI2Cx->I2C_CCR = tmp;

	tmp = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {

		tmp = (RCC_GetPLCK1Value() / 1000000U ) + 1;
	} else {
		tmp = ((RCC_GetPLCK1Value() * 300) / 1000000000U ) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tmp & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t* pI2CHandle) {

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t flagName) {
	if (pI2Cx->I2C_SR1 & flagName) return FLAG_SET;
	return FLAG_RESET;
}

void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE_POS);
	} else {
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE_POS);
	}
}

void I2C_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4, iprx_section = IRQNumber % 4;
	uint8_t shift_num = (8 * iprx_section) + (8 - NO_PR_BITS_IMPL);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_num);
}

__weak__ void I2C_ApplicationEventCallback(I2C_Handle_t* pI2CHandle, uint8_t I2C_EVENT) {

}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t dummyRead;
	if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL_POS)) {
		// device in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if (pI2CHandle->RxSize == 1) {
				I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);
				dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
				dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
				(void) dummyRead;
			}
		} else {
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
			(void) dummyRead;
		}
	} else {
		// device in slave mode
		dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
		dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
		(void) dummyRead;
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx) {
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START_POS);

}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr) {
	slaveAddr = (slaveAddr << 1);
	slaveAddr &= ~(1);
	pI2Cx->I2C_DR = slaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr) {
	slaveAddr = (slaveAddr << 1);
	slaveAddr |= 1;
	pI2Cx->I2C_DR = slaveAddr;
}

void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx) {
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP_POS);
}

void I2C_ManageACKing(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK_POS);
	} else {
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK_POS);
	}
}

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr) {
	// Create start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// confirm start generation is completed by checking SB flag
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));
	// send address of the slave with r/nw bit set to 0
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
	// check if addr has been set
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	// clear addr flag according to software sequence to stop clock stretching
	I2C_ClearADDRFlag(pI2CHandle);
	// send the data
	while (len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}
	// make sure TXE = 1 and BTF = 1 before generating stop condition
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));
	// create stop condition
	if (Sr == I2C_DISABLE_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr) {
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	if (len == 1) {
		I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);
		I2C_ClearADDRFlag(pI2CHandle);
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));
		if (Sr == I2C_DISABLE_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
	}
	if (len > 1) {
		I2C_ClearADDRFlag(pI2CHandle);
		for (uint32_t i = len; i > 0; i--) {
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));
			if (i == 2) {
				I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);
				if (Sr == I2C_DISABLE_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pRxBuffer++;
		}
	}
	if (pI2CHandle->I2C_Config.I2C_ACKControl) I2C_ManageACKing(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data) {
	pI2Cx->I2C_DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx) {
	return (uint8_t)pI2Cx->I2C_DR;
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr) {
	uint8_t busyState = pI2CHandle->TxRxState;
	if (((busyState != I2C_BUSY_IN_RX) && (busyState != I2C_BUSY_IN_TX))) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN_POS);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN_POS);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN_POS);
	}
	return busyState;
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle) {
	if (pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2) {
			I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);
		}
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0) {
		if (pI2CHandle->Sr == I2C_DISABLE_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		I2C_CloseReceiveData(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle) {
	if (pI2CHandle->TxLen > 0) {
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}

void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle) {
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN_POS);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN_POS);
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if (pI2CHandle->I2C_Config.I2C_ACKControl) I2C_ManageACKing(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_CloseSendData(I2C_Handle_t* pI2CHandle) {
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN_POS);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN_POS);
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr) {
	uint8_t busyState = pI2CHandle->TxRxState;
	if (((busyState != I2C_BUSY_IN_RX) && (busyState != I2C_BUSY_IN_TX))) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->RxSize = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN_POS);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN_POS);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN_POS);
	}
	return busyState;
}

void I2C_SlaveEnOrDiCallbackEvents(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi) {
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN_POS);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN_POS);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN_POS);
	} else {
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN_POS);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN_POS);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN_POS);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle) {
	uint32_t tmp1, tmp2, tmp3;
	tmp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN_POS);
	tmp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN_POS);
	tmp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB_POS);
	if (tmp1 && tmp3) {
		// sb flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}
	tmp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR_POS);
	if (tmp1 && tmp3) {
		// addr flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}
	tmp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF_POS);
	if (tmp1 && tmp3) {
		// btf flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			if (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE_POS)) {
				// btf and txe are set
				if (pI2CHandle->TxLen == 0) {
					// generate stop condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR) I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					// reset member elements of handle struct
					I2C_CloseSendData(pI2CHandle);
					// notify application about tx complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			;
		}
	}
	tmp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF_POS);
	if (tmp1 && tmp3) {
		// stop flag is set
		// clear stop flag by reading sr1 and writing cr1
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	tmp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE_POS);
	if (tmp1 && tmp2 && tmp3) {
		// txe flag is set
		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL_POS)) {
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		} else {
			// slave received request for data
			if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA_POS)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}
	tmp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RXNE_POS);
	if (tmp1 && tmp2 && tmp3) {
		// rxne flag is set
		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL_POS)) {
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else {
			if (!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA_POS))) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle) {
	uint32_t temp1,temp2;
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN_POS);
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR_POS);
	if(temp1  && temp2 )
	{
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR_POS);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO_POS );
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO_POS);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF_POS);
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF_POS);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR_POS);
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR_POS);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT_POS);
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT_POS);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}
