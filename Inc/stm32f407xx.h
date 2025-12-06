/*
 * stm32f407xx.h
 *
 *  Created on: Nov 16, 2025
 *      Author: Blah
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak__ __attribute__((weak))

// Processor specific details
#define NVIC_ISER0 ((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1 ((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2 ((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3 ((__vo uint32_t*)0xE000E10CU)

#define NVIC_ICER0 ((__vo uint32_t*)0xE000E180U)
#define NVIC_ICER1 ((__vo uint32_t*)0xE000E184U)
#define NVIC_ICER2 ((__vo uint32_t*)0xE000E188U)
#define NVIC_ICER3 ((__vo uint32_t*)0xE000E18CU)

#define NVIC_IPR_BASE_ADDR ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPL 4

// Base addresses of memory regions
#define FLASH_BASE_ADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR 0x1FFF0000U
#define SRAM SRAM1_BASEADDR

// AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASE 0x40000000U
#define APB1_PERIPH_BASE PERIPH_BASE
#define APB2_PERIPH_BASE 0x40010000U
#define AHB1_PERIPH_BASE 0x40020000U
#define AHB2_PERIPH_BASE 0x50000000U

// AHB1 Bus Peripherals
#define GPIOA_BASE_ADDR (AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASE_ADDR (AHB1_PERIPH_BASE + 0x0400)
#define GPIOC_BASE_ADDR (AHB1_PERIPH_BASE + 0x0800)
#define GPIOD_BASE_ADDR (AHB1_PERIPH_BASE + 0x0C00)
#define GPIOE_BASE_ADDR (AHB1_PERIPH_BASE + 0x1000)
#define GPIOF_BASE_ADDR (AHB1_PERIPH_BASE + 0x1400)
#define GPIOG_BASE_ADDR (AHB1_PERIPH_BASE + 0x1800)
#define GPIOH_BASE_ADDR (AHB1_PERIPH_BASE + 0x1C00)
#define GPIOI_BASE_ADDR (AHB1_PERIPH_BASE + 0x2000)
#define GPIOJ_BASE_ADDR (AHB1_PERIPH_BASE + 0x2400)
#define GPIOK_BASE_ADDR (AHB1_PERIPH_BASE + 0x2800)
#define RCC_BASE_ADDR (AHB1_PERIPH_BASE + 0x3800)

// APB1 Bus Peripherals
#define I2C1_BASE_ADDR (APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASE_ADDR (APB1_PERIPH_BASE + 0x5800)
#define I2C3_BASE_ADDR (APB1_PERIPH_BASE + 0x5C00)
#define SPI2_BASE_ADDR (APB1_PERIPH_BASE + 0x3800)
#define SPI3_BASE_ADDR (APB1_PERIPH_BASE + 0x3C00)
#define USART2_BASE_ADDR (APB1_PERIPH_BASE + 0x4400)
#define USART3_BASE_ADDR (APB1_PERIPH_BASE + 0x4800)
#define UART4_BASE_ADDR (APB1_PERIPH_BASE + 0x4C00)
#define UART5_BASE_ADDR (APB1_PERIPH_BASE + 0x5000)

// APB2 Bus Peripherals
#define SPI1_BASE_ADDR (APB2_PERIPH_BASE + 0x3000)
#define SPI4_BASE_ADDR (APB2_PERIPH_BASE + 0x3400)
#define USART1_BASE_ADDR (APB2_PERIPH_BASE + 0x1000)
#define USART6_BASE_ADDR (APB2_PERIPH_BASE + 0x1400)
#define EXTI_BASE_ADDR (APB2_PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE_ADDR (APB2_PERIPH_BASE + 0x3800)


// Peripheral Register Definition Structs

typedef struct {
	__vo uint32_t MODER; // Configure the I/O direction mode. 00 = Input, 01 = GP Output mode, 10 = Alternate Function Mode, 11 = Analog Mode
	__vo uint32_t OTYPER; // Configure the output type of the I/O port. 0 = Push-pull, 1 = Open drain
	__vo uint32_t OSPEEDR; // Configure the I/O output speed. 00 = Low speed, 01 = Medium speed, 10 = High Speed, 11 = Very high speed
	__vo uint32_t PUPDR; // Configure I/O pull up or pull down. 00 = No pull up or pull down, 01 = Pull up, 10 = Pull down, 11 = Reserved
	__vo uint32_t IDR; // Contains the input value of the corresponding I/O port. Read only.
	__vo uint32_t ODR; // Contains the output value of the corresponding I/O port.
	__vo uint32_t BSRR; // 15:0 Writing 1 sets the ODR bit of the corresponding I/O port. 16:31 Writing 1 resets the ODR bit of the corresponding I/O port.
	__vo uint32_t LCKR; // 15:0 Writing 1 locks the corresponding I/O port configuration. Bit 16 serves as lock key. 0 = Port configuration key not active, 1 = Port configuration key active (GPIOx_LCKR register is locked until MCU reset or peripheral reset occurs.) 31:17 Reserved.
	__vo uint32_t AFRL; // Sets the alternate function mode of first 8 GPIO ports.
	__vo uint32_t AFRH; // Sets the alternate function mode of last 8 GPIO ports.
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t __RES1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t __RES2;
	uint32_t __RES3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2NER;
	__vo uint32_t AHB3ENR;
	uint32_t __RES4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t __RES5;
	uint32_t __RES6;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t __RES7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t __RES8;
	uint32_t __RES9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t __RES10;
	uint32_t __RES11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SSCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
} RCC_RegDef_t;


typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;


typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR1;
	__vo uint32_t EXTICR2;
	__vo uint32_t EXTICR3;
	__vo uint32_t EXTICR4;
	__vo uint32_t CMPCR;
} SYSCFG_RegDef_t;


typedef struct {
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
} SPI_RegDef_t;


typedef struct {
	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_DR;
	__vo uint32_t I2C_SR1;
	__vo uint32_t I2C_SR2;
	__vo uint32_t I2C_CCR;
	__vo uint32_t I2C_TRISE;
	__vo uint32_t I2C_FLTR;
} I2C_RegDef_t;


typedef struct {
	__vo uint32_t USART_SR;
	__vo uint32_t USART_DR;
	__vo uint32_t USART_BRR;
	__vo uint32_t USART_CR1;
	__vo uint32_t USART_CR2;
	__vo uint32_t USART_CR3;
	__vo uint32_t USART_GTPR;
} USART_RegDef_t;

// Peripheral Definitions

#define RCC ((RCC_RegDef_t*) RCC_BASE_ADDR)

#define EXTI ((EXTI_RegDef_t*) EXTI_BASE_ADDR)

#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_RegDef_t*) GPIOI_BASE_ADDR)



#define SPI1 ((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2 ((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3 ((SPI_RegDef_t*) SPI3_BASE_ADDR)
#define SPI4 ((SPI_RegDef_t*) SPI4_BASE_ADDR)


#define I2C1 ((I2C_RegDef_t*) I2C1_BASE_ADDR)
#define I2C2 ((I2C_RegDef_t*) I2C2_BASE_ADDR)
#define I2C3 ((I2C_RegDef_t*) I2C3_BASE_ADDR)



#define USART1 ((USART_RegDef_t*) USART1_BASE_ADDR)
#define USART2 ((USART_RegDef_t*) USART2_BASE_ADDR)
#define USART3 ((USART_RegDef_t*) USART3_BASE_ADDR)
#define USART6 ((USART_RegDef_t*) USART6_BASE_ADDR)

// Clock enable macros for GPIOx peripherals
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))

// Clock enable macros for I2C peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

// Clock enable for SPIx peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

// Clock enable for USARTx peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))

// Clock enable for SYSCFG peripheral
#define SYSCFGEN_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))

// Clock disable macros for I2C peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

// Clock disable macros for SPI peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))

// Clock disable macros for USART peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))

// Clock disable macros for SYSCFG peripheral
#define SYSCFGEN_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

// Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA)?0:\
								 (x == GPIOB)?1:\
								 (x == GPIOC)?2:\
								 (x == GPIOD)?3:\
								 (x == GPIOE)?4:\
								 (x == GPIOF)?5:\
								 (x == GPIOG)?6:\
								 (x == GPIOH)?7:0)
// Generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET RESET
#define FLAG_SET SET
// IRQ Nums for EXTI NVIC
#define EXTI0 6
#define EXTI1 7
#define EXTI2 8
#define EXTI3 9
#define EXTI4 10
#define EXTI9_5 23
#define EXTI15_10 40

// IRQ Nums for SPI peripherals
#define IRQ_SPI1 35
#define IRQ_SPI2 36
#define IRQ_SPI3 51

// IRQ Priorities
#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI1 1
#define NVIC_IRQ_PRI2 2
#define NVIC_IRQ_PRI3 3
#define NVIC_IRQ_PRI4 4
#define NVIC_IRQ_PRI5 5
#define NVIC_IRQ_PRI6 6
#define NVIC_IRQ_PRI7 7
#define NVIC_IRQ_PRI8 8
#define NVIC_IRQ_PRI9 9
#define NVIC_IRQ_PRI10 10
#define NVIC_IRQ_PRI11 11
#define NVIC_IRQ_PRI12 12
#define NVIC_IRQ_PRI13 13
#define NVIC_IRQ_PRI14 14
#define NVIC_IRQ_PRI15 15

// CR1
#define SPI_CR1_CPHA_POS 0
#define SPI_CR1_CPOL_POS 1
#define SPI_CR1_MSTR_POS 2
#define SPI_CR1_BR_POS 3
#define SPI_CR1_SPE_POS 6
#define SPI_CR1_LSBFIRST 7
#define SPI_CR1_SSI_POS 8
#define SPI_CR1_SSM_POS 9
#define SPI_CR1_RXONLY_POS 10
#define SPI_CR1_DFF_POS 11
#define SPI_CR1_CRCNEXT 12
#define SPI_CR1_CRCEN 13
#define SPI_CR1_BIDIOE_POS 14
#define SPI_CR1_BIDIMODE_POS 15

// CR2
#define SPI_CR2_RXDMAEN_POS 0
#define SPI_CR2_TXDMAEN_POS 1
#define SPI_CR2_SSOE_POS 2
#define SPI_CR2_FRF_POS 4
#define SPI_CR2_ERRIE_POS 5
#define SPI_CR2_RXNEIE_POS 6
#define SPI_CR2_TXEIE_POS 7

// SR
#define SPI_SR_RXNE_POS 0
#define SPI_SR_TXE_POS 1
#define SPI_SR_CHSIDE_POS 2
#define SPI_SR_UDR_POS 3
#define SPI_SR_CRCERR_POS 4
#define SPI_SR_MODF_POS 5
#define SPI_SR_OVR_POS 6
#define SPI_SR_BSY_POS 7
#define SPI_SR_FRE_POS 8

#define I2C_CR1_PE_POS 0
#define I2C_CR1_SMBUS_POS 1
#define I2C_CR1_SMBTYPE_POS 3
#define I2C_CR1_ENARP_POS 4
#define I2C_CR1_ENPEC_POS 5
#define I2C_CR1_ENGC_POS 6
#define I2C_CR1_NOSTRETCH_POS 7
#define I2C_CR1_START_POS 8
#define I2C_CR1_STOP_POS 9
#define I2C_CR1_ACK_POS 10
#define I2C_CR1_POS_POS 11
#define I2C_CR1_PEC_POS 12
#define I2C_CR1_ALERT_POS 13
#define I2C_CR1_SWRST_POS 15

#define I2C_CR2_ITERREN_POS 8
#define I2C_CR2_ITEVTEN_POS 9
#define I2C_CR2_ITBUFEN_POS 10
#define I2C_CR2_DMAEN_POS 11
#define I2C_CR2_LAST_POS 12

#define I2C_OAR1_ENDUAL_POS 0

#define I2C_SR1_SB_POS 0
#define I2C_SR1_ADDR_POS 1
#define I2C_SR1_BTF_POS 2
#define I2C_SR1_ADD10_POS 3
#define I2C_SR1_STOPF_POS 4
#define I2C_SR1_RXNE_POS 6
#define I2C_SR1_TXE_POS 7
#define I2C_SR1_BERR_POS 8
#define I2C_SR1_ARLO_POS 9
#define I2C_SR1_AF_POS 10
#define I2C_SR1_OVR_POS 11
#define I2C_SR1_PECERR_POS 12
#define I2C_SR1_TIMEOUT_POS 14
#define I2C_SR1_SMBALERT_POS 15

#define I2C_SR2_MSL_POS 0
#define I2C_SR2_BUSY_POS 1
#define I2C_SR2_TRA_POS 2
#define I2C_SR2_GENCALL_POS 4
#define I2C_SR2_SMBDEFAULT_POS 5
#define I2C_SR2_SMBHOST_POS 6
#define I2C_SR2_DUALF_POS 7

#define I2C_CCR_DUTY_POS 14
#define I2C_CCR_FS_POS 15

#define I2C_FLTR_ANOFF_POS 4

#define IRQ_I2C1_EV 31
#define IRQ_I2C1_ER 32
#define IRQ_I2C2_EV 33
#define IRQ_I2C2_ER 34
#define IRQ_I2C3_EV 72
#define IRQ_I2C3_ER 73



#define USART_SR_PE_POS 0
#define USART_SR_FE_POS 1
#define USART_SR_NF_POS 2
#define USART_SR_ORE_POS 3
#define USART_SR_IDLE_POS 4
#define USART_SR_RXNE_POS 5
#define USART_SR_TC_POS 6
#define USART_SR_TXE_POS 7
#define USART_SR_LBD_POS 8
#define USART_SR_CTS_POS 9

#define USART_CR1_SBK_POS 0
#define USART_CR1_RWU_POS 1
#define USART_CR1_RE_POS 2
#define USART_CR1_TE_POS 3
#define USART_CR1_IDLEIE_POS 4
#define USART_CR1_RXNEIE_POS 5
#define USART_CR1_TCIE_POS 6
#define USART_CR1_TXEIE_POS 7
#define USART_CR1_PEIE_POS 8
#define USART_CR1_PS_POS 9
#define USART_CR1_PCE_POS 10
#define USART_CR1_WAKE_POS 11
#define USART_CR1_M_POS 12
#define USART_CR1_UE_POS 13
#define USART_CR1_OVER8_POS 15

#define USART_CR2_LBDL_POS 5
#define USART_CR2_LBDIE_POS 6
#define USART_CR2_LBCL_POS 8
#define USART_CR2_CPHA_POS 9
#define USART_CR2_CPOL_POS 10
#define USART_CR2_CLKEN_POS 11
#define USART_CR2_STOP_POS 12
#define USART_CR2_LINEN_POS 14

#define USART_CR3_EIE_POS 0
#define USART_CR3_IREN_POS 1
#define USART_CR3_IRLP_POS 2
#define USART_CR3_HDSEL_POS 3
#define USART_CR3_NACK_POS 4
#define USART_CR3_SCEN_POS 5
#define USART_CR3_DMAR_POS 6
#define USART_CR3_DMAT_POS 7
#define USART_CR3_RTSE_POS 8
#define USART_CR3_CTSE_POS 9
#define USART_CR3_CTSIE_POS 10
#define USART_CR3_ONEBIT_POS 11


#define IRQ_USART1 37
#define IRQ_USART2 38
#define IRQ_USART3 39
#define IRQ_UART4 40
#define IRQ_UART5 41
#define IRQ_USART6 71

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
