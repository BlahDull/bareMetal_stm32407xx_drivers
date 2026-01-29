/*
 * stm32f407xx_dma_driver.h
 *
 *  Created on: Jan 23, 2026
 *      Author: Blah
 */

#ifndef STM32F407XX_DMA_DRIVER_H_
#define STM32F407XX_DMA_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint16_t DMA_Channel;
	uint16_t DMA_Stream;
	uint8_t DMA_Direction;
	uint8_t DMA_DataWidth;
	uint8_t DMA_FifoMode;
	uint8_t DMA_FifoThreshold;
	uint8_t DMA_Mode;
	uint8_t DMA_Priority;
	uint8_t DMA_MemAutoIncr;
	uint8_t DMA_PeriphAutoIncr;
	uint16_t DMA_DataLen;
	uint32_t DMA_Src;
	uint32_t DMA_Dst;
} DMA_Config_t;

typedef struct {
	DMA_RegDef_t* pDMA;
	DMA_Config_t  DMA_Config;
} DMA_Handle_t;

#define DMA_Channel_0 0b000U
#define DMA_Channel_1 0b001U
#define DMA_Channel_2 0b010U
#define DMA_Channel_3 0b011U
#define DMA_Channel_4 0b100U
#define DMA_Channel_5 0b101U
#define DMA_Channel_6 0b110U
#define DMA_Channel_7 0b111U

#define DMA_Stream_0 0b0000U
#define DMA_Stream_1 0b0001U
#define DMA_Stream_2 0b0010U
#define DMA_Stream_3 0b0011U
#define DMA_Stream_4 0b0100U
#define DMA_Stream_5 0b0101U
#define DMA_Stream_6 0b0110U
#define DMA_Stream_7 0b0111U

#define DMA_MEM_TO_MEM 0b11U
#define DMA_PERIPH_TO_MEM 0b00U
#define DMA_MEM_TO_PERIPH 0b01U

#define DMA_DATAWIDTH_HALFWORD 0b01U
#define DMA_DATAWIDTH_WORD 0b10U
#define DMA_DATAWIDTH_BYTE 0b00U

#define DMA_FIFOMODE_ENABLE 0b01U
#define DMA_FIFOMODE_DISABLE 0b00U

#define DMA_FIFOSTATUS_LESS_ONEFOURTH 0b000U
#define DMA_FIFOSTATUS_GREATER_ONEFOURTH_LESS_ONEHALF 0b001U
#define DMA_FIFOSTATUS_GREATER_ONEHALF_LESS_THREEFOURTH 0b010U
#define DMA_FIFOSTATUS_GREATER_THREEFOURTH_LESS_FULL 0b011U
#define DMA_FIFOSTATUS_EMPTY 0b100U
#define DMA_FIFOSTATUS_FULL 0b101U

#define DMA_FIFOTHRESH_ONEFOURTH 0b00U
#define DMA_FIFOTHRESH_ONEHALF 0b01U
#define DMA_FIFOTHRESH_THREEFOURTH 0b10U
#define DMA_FIFOTHRESH_FULL 0b11U

#define DMA_MODE_NORMAL 0b00U
#define DMA_MODE_CIRCULAR 0b01U

#define DMA_PRIORITY_LOW 0b00U
#define DMA_PRIORITY_MED 0b01U
#define DMA_PRIORITY_HIGH 0b10U
#define DMA_PRIORITY_VHIGH 0b11U

#define DMA_MEM_AUTOINCR_ENABLE 0b1U
#define DMA_MEM_AUTOINCR_DISABLE 0b0U

#define DMA_PERIPH_AUTOINCR_ENABLE 0b1U
#define DMA_PERIPH_AUTOINCR_DISABLE 0b0U

#define DMA_IT_HT  0b00001U
#define DMA_IT_TC  0b00010U
#define DMA_IT_TE  0b00100U
#define DMA_IT_FE  0b01000U
#define DMA_IT_DME 0b10000U

void DMA_Init(DMA_Handle_t*, uint8_t);
void DMA_IRQHandler(DMA_Handle_t*);
void DMA_EnableStream(DMA_Handle_t*);
void HT_Complete_Callback(uint8_t);
void TC_Complete_Callback(uint8_t);
void TE_Error_Callback(uint8_t);
void FE_Error_Callback(uint8_t);
void DME_Error_Callback(uint8_t);
#endif /* STM32F407XX_DMA_DRIVER_H_ */
