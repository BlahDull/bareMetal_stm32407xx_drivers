/*
 * stm32f407xx_dma_driver.c
 *
 *  Created on: Jan 23, 2026
 *      Author: Blah
 */

#include "stm32f407xx_dma_driver.h"

static void DMA_PeriClockControl(DMA_RegDef_t* DMA, uint8_t EnOrDi) {
	if (DMA == DMA1) {
		DMA1_PCLK_EN();
	} else {
		DMA2_PCLK_EN();
	}
}

void DMA_EnableStream(DMA_Handle_t* hDMA) {
	DMA_Stream_t* currentStream = &(hDMA->pDMA->DMA_Streams[hDMA->DMA_Config.DMA_Stream]);
	currentStream->DMA_S_CR &= ~(1);
	currentStream->DMA_S_CR |= (1);
}

static void DMA_ConfigureStream(DMA_Handle_t* hDMA) {
	DMA_Stream_t* currentStream = &(hDMA->pDMA->DMA_Streams[hDMA->DMA_Config.DMA_Stream]);
	// select channel
	currentStream->DMA_S_CR &= ~(0x7 << 25);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_Channel << 25);
	// set data transfer direction
	currentStream->DMA_S_CR &= ~(0x3 << 6);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_Direction << 6);
	// set number of data
	currentStream->DMA_S_NDTR &= ~(0xFFFF);
	currentStream->DMA_S_NDTR |= (hDMA->DMA_Config.DMA_DataLen);
	// set data size
		// set memory data size
	currentStream->DMA_S_CR &= ~(0x3 << 13);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_DataWidth << 13);
		// set peripheral data size
	currentStream->DMA_S_CR &= ~(0x3 << 11);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_DataWidth << 11);
	// set mode
	currentStream->DMA_S_CR &= ~(1 << 8);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_Mode << 8);
	// set fifo mode
	currentStream->DMA_S_FCR &= ~(1 << 2);
	currentStream->DMA_S_FCR |= (hDMA->DMA_Config.DMA_FifoMode << 2);
	// set fifo threshold
	currentStream->DMA_S_FCR &= ~(0x3);
	currentStream->DMA_S_FCR |= (hDMA->DMA_Config.DMA_FifoThreshold);
	// set priority
	currentStream->DMA_S_CR &= ~(0x3 << 16);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_Priority << 16);
	// set auto increments
	currentStream->DMA_S_CR &= ~(1 << 10);
	currentStream->DMA_S_CR &= ~(1 << 9);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_MemAutoIncr << 10);
	currentStream->DMA_S_CR |= (hDMA->DMA_Config.DMA_PeriphAutoIncr << 9);
	// set addresses
	currentStream->DMA_S_M0AR &= ~(0xFFFFFFFF);
	currentStream->DMA_S_M1AR &= ~(0xFFFFFFFF);
	currentStream->DMA_S_PAR &= ~(0xFFFFFFFF);
	switch (hDMA->DMA_Config.DMA_Direction) {
	case DMA_MEM_TO_PERIPH:
		currentStream->DMA_S_PAR = hDMA->DMA_Config.DMA_Dst;
		currentStream->DMA_S_M0AR = hDMA->DMA_Config.DMA_Src;
		break;
	case DMA_PERIPH_TO_MEM:
		currentStream->DMA_S_PAR = hDMA->DMA_Config.DMA_Src;
		currentStream->DMA_S_M0AR = hDMA->DMA_Config.DMA_Dst;
		break;
	case DMA_MEM_TO_MEM:
		currentStream->DMA_S_M0AR = hDMA->DMA_Config.DMA_Src;
		currentStream->DMA_S_M1AR = hDMA->DMA_Config.DMA_Dst;
		break;
	}
	// enable stream
	DMA_EnableStream(hDMA);
}

static void DMA_IRQ_Config(uint8_t IRQNumber, uint8_t EnOrDi) {
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

void DMA_ConfigInterrupt(DMA_Handle_t* hDMA, uint8_t interrupts) {
	if (hDMA->pDMA == DMA1) {
		switch (hDMA->DMA_Config.DMA_Stream) {
		case 0:
			DMA_IRQ_Config(IRQ_DMA1_STREAM0, ENABLE);
			break;
		case 1:
			DMA_IRQ_Config(IRQ_DMA1_STREAM1, ENABLE);
			break;
		case 2:
			DMA_IRQ_Config(IRQ_DMA1_STREAM2, ENABLE);
			break;
		case 3:
			DMA_IRQ_Config(IRQ_DMA1_STREAM3, ENABLE);
			break;
		case 4:
			DMA_IRQ_Config(IRQ_DMA1_STREAM4, ENABLE);
			break;
		case 5:
			DMA_IRQ_Config(IRQ_DMA1_STREAM5, ENABLE);
			break;
		case 6:
			DMA_IRQ_Config(IRQ_DMA1_STREAM6, ENABLE);
			break;
		}
	} else if (hDMA->pDMA == DMA2) {
		switch (hDMA->DMA_Config.DMA_Stream) {
		case 0:
			DMA_IRQ_Config(IRQ_DMA2_STREAM0, ENABLE);
			break;
		case 1:
			DMA_IRQ_Config(IRQ_DMA2_STREAM1, ENABLE);
			break;
		case 2:
			DMA_IRQ_Config(IRQ_DMA2_STREAM2, ENABLE);
			break;
		case 3:
			DMA_IRQ_Config(IRQ_DMA2_STREAM3, ENABLE);
			break;
		case 4:
			DMA_IRQ_Config(IRQ_DMA2_STREAM4, ENABLE);
			break;
		}
	}
	DMA_Stream_t* currentStream = &(hDMA->pDMA->DMA_Streams[hDMA->DMA_Config.DMA_Stream]);
	if (interrupts & 0b1) {
		currentStream->DMA_S_CR |= (1 << 3);
	}
	if ((interrupts >> 1) & 0b1) {
		currentStream->DMA_S_CR |= (1 << 4);
	}
	if ((interrupts >> 2) & 0b1) {
		currentStream->DMA_S_CR |= (1 << 2);
	}
	if ((interrupts >> 3) & 0b1) {
		currentStream->DMA_S_FCR |= (1 << 7);
	}
	if ((interrupts >> 4) & 0b1) {
		currentStream->DMA_S_CR |= (1 << 1);
	}
}

void DMA_Init(DMA_Handle_t* hDMA, uint8_t interrupts) {
	DMA_PeriClockControl(hDMA->pDMA, ENABLE);
	if (interrupts != 0)
		DMA_ConfigInterrupt(hDMA, interrupts);
	DMA_ConfigureStream(hDMA);
}

void DMA_IRQHandler(DMA_Handle_t* hDMA) {
	DMA_Stream_t* currentStream = &(hDMA->pDMA->DMA_Streams[hDMA->DMA_Config.DMA_Stream]);
	switch (hDMA->DMA_Config.DMA_Stream) {
	case 0:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 4) &  1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 4);
			HT_Complete_Callback(0);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_LISR >> 5) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 5);
			TC_Complete_Callback(0);
		} else if ((currentStream->DMA_S_CR >> 5) & (hDMA->pDMA->DMA_LISR >> 3) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 3);
			TE_Error_Callback(0);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_LISR >> 0) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 0);
			FE_Error_Callback(0);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 2) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 2);
			DME_Error_Callback(0);
		}
		break;
	case 1:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 10) &  1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 10);
			HT_Complete_Callback(1);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_LISR >> 11) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 11);
			TC_Complete_Callback(1);
		} else if ((currentStream->DMA_S_CR >> 2) & (hDMA->pDMA->DMA_LISR >> 9) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 9);
			TE_Error_Callback(1);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_LISR >> 6) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 6);
			FE_Error_Callback(1);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 8) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 8);
			DME_Error_Callback(1);
		}
		break;
	case 2:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 20) &  1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 20);
			HT_Complete_Callback(2);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_LISR >> 21) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 21);
			TC_Complete_Callback(2);
		} else if ((currentStream->DMA_S_CR >> 2) & (hDMA->pDMA->DMA_LISR >> 19) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 19);
			TE_Error_Callback(2);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_LISR >> 16) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 16);
			FE_Error_Callback(2);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 18) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 18);
			DME_Error_Callback(2);
		}
		break;
	case 3:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 26) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 26);
			HT_Complete_Callback(3);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_LISR >> 27) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 27);
			TC_Complete_Callback(3);
		} else if ((currentStream->DMA_S_CR >> 2) & (hDMA->pDMA->DMA_LISR >> 25) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 25);
			TE_Error_Callback(3);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_LISR >> 22) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 22);
			FE_Error_Callback(3);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_LISR >> 24) & 1) {
			hDMA->pDMA->DMA_LIFCR |= (1 << 24);
			DME_Error_Callback(3);
		}
		break;
	case 4:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 4) &  1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 4);
			HT_Complete_Callback(4);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_HISR >> 5) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 5);
			TC_Complete_Callback(4);
		} else if ((currentStream->DMA_S_CR >> 2) & (hDMA->pDMA->DMA_HISR >> 3) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 3);
			TE_Error_Callback(4);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_HISR >> 0) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 0);
			FE_Error_Callback(4);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 2) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 2);
			DME_Error_Callback(4);
		}
		break;
	case 5:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 10) &  1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 10);
			HT_Complete_Callback(5);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_HISR >> 11) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 11);
			TC_Complete_Callback(5);
		} else if ((currentStream->DMA_S_CR >> 2) & (hDMA->pDMA->DMA_HISR >> 9) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 9);
			TE_Error_Callback(5);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_HISR >> 6) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 6);
			FE_Error_Callback(5);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 8) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 8);
			DME_Error_Callback(5);
		}
		break;
	case 6:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 20) &  1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 20);
			HT_Complete_Callback(6);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_HISR >> 21) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 21);
			TC_Complete_Callback(6);
		} else if ((currentStream->DMA_S_CR >> 2) & (hDMA->pDMA->DMA_HISR >> 19) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 19);
			TE_Error_Callback(6);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_HISR >> 16) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 16);
			FE_Error_Callback(6);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 18) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 18);
			DME_Error_Callback(6);
		}
		break;
	case 7:
		if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 26) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 26);
			HT_Complete_Callback(7);
		} else if ((currentStream->DMA_S_CR >> 4) & (hDMA->pDMA->DMA_HISR >> 27) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 27);
			TC_Complete_Callback(7);
		} else if ((currentStream->DMA_S_CR >> 2) & (hDMA->pDMA->DMA_HISR >> 25) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 25);
			TE_Error_Callback(7);
		} else if ((currentStream->DMA_S_FCR >> 7) & (hDMA->pDMA->DMA_HISR >> 22) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 22);
			FE_Error_Callback(7);
		} else if ((currentStream->DMA_S_CR >> 3) & (hDMA->pDMA->DMA_HISR >> 24) & 1) {
			hDMA->pDMA->DMA_HIFCR |= (1 << 24);
			DME_Error_Callback(7);
		}
		break;
	}
}

__weak__ void HT_Complete_Callback(uint8_t) {}
__weak__ void TC_Complete_Callback(uint8_t) {}
__weak__ void TE_Error_Callback(uint8_t) {}
__weak__ void FE_Error_Callback(uint8_t) {}
__weak__ void DME_Error_Callback(uint8_t) {}
