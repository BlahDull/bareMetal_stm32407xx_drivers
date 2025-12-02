/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 20, 2025
 *      Author: Blah
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t* pSP;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxState;
	uint8_t RxState;
	uint32_t TxLen;
	uint32_t RxLen;
} SPI_Handle_t;

#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

#define SPI_BUS_CONFIG_FD 1
#define SPI_BUS_CONFIG_HD 2
#define SPI_BUS_CONFIG_SIMP_RXONLY 4

#define SPI_SCLK_SPEED_DIV2 0
#define SPI_SCLK_SPEED_DIV4 1
#define SPI_SCLK_SPEED_DIV8 2
#define SPI_SCLK_SPEED_DIV16 3
#define SPI_SCLK_SPEED_DIV32 4
#define SPI_SCLK_SPEED_DIV64 5
#define SPI_SCLK_SPEED_DIV128 6
#define SPI_SCLK_SPEED_DIV256 7

#define SPI_DFF_8BIT 0
#define SPI_DFF_16BIT 1

#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

#define SPI_SSM_DI 0
#define SPI_SSM_EN 1

#define SPI_TXE_FLAG (1 << SPI_SR_TXE_POS)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE_POS)
#define SPI_BSY_FLAG (1 << SPI_SR_BSY_POS)

// SPI states
#define SPI_READY 0
#define SPI_BUSY_RX 1
#define SPI_BUSY_TX 2
// SPI events
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR  3

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnOrDi);
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIHandle);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTXBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRXBuffer, uint32_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pTXBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pRXBuffer, uint32_t len);
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi);
void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_ClearOVRFlag(SPI_RegDef_t* pSPIx);
void SPI_CloseTransmission(SPI_Handle_t* pHandle);
void SPI_CloseReception(SPI_Handle_t* pHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle, uint8_t SPI_EVENT);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
