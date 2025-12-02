/*
 * stm32f07xx_i2c_driver.h
 *
 *  Created on: Nov 26, 2025
 *      Author: Blah
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {
	I2C_RegDef_t* pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t Sr;
} I2C_Handle_t;


#define I2C_READY 0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2

#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM4k 200000

#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

#define I2C_DISABLE_SR RESET
#define I2C_ENABLE_SR SET

#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

#define I2C_EV_STOP 0
#define I2C_EV_TX_CMPLT 1
#define I2C_EV_RX_CMPLT 2
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7
#define I2C_EV_DATA_REQ 8
#define I2C_EV_DATA_RCV 9

#define I2C_TXE_FLAG (1 << I2C_SR1_TXE_POS)
#define I2C_RXNE_FLAG (1 << I2C_SR1_RXNE_POS)
#define I2C_SB_FLAG (1 << I2C_SR1_SB_POS)
#define I2C_BTF_FLAG (1 << I2C_SR1_BTF_POS)
#define I2C_STOPF_FLAG (1 << I2C_SR1_STOPF_POS)
#define I2C_BERR_FLAG (1 << I2C_SR1_BERR_POS)
#define I2C_ARLO_FLAG (1 << I2C_SR1_ARLO_POS)
#define I2C_AF_FLAG (1 << I2C_SR1_AF_POS)
#define I2C_OVR_FLAG (1 << I2C_SR1_OVR_POS)
#define I2C_TIMEOUT_FLAG (1 << I2C_SR1_TIMEOUT_POS)
#define I2C_ADDR_FLAG (1 << I2C_SR1_ADDR_POS)

void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi);
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2CHandle);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi);
void I2C_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_ApplicationEventCallback(I2C_Handle_t* pI2CHandle, uint8_t I2C_EVENT);
void I2C_ManageACKing(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi);
void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t* pI2CHandle);
void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
void I2C_SlaveEnOrDiCallbackEvents(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi);
#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
