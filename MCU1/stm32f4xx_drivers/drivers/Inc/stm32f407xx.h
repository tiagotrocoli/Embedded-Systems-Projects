/*
 * stm32f407xx.h
 *
 *  Created on: May 8, 2020
 *      Author: tiago
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile

/*********************************************START: Processor Specific Details*******************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0				((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor NVIC Priority register Addresses
 */

#define NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED	4
/*
 * Base address of flash and SRAM memory
 */

#define FLASH_BASEADDR			0x08000000U /*It's where code are stored*/
#define SRAM1_BASEADDR			0x20000000U /*112Kb of space*/
#define SRAM2_BASEADDR			0x20001C00U /* (112Kb)*1024 = 0x1C00 */
#define ROM_BASEADDR			0x1FFF0000U /*System memory*/
#define SRAM					SRAM1_BASEADDR

/*
 *  AHBx and APBx Bus Peripheral base address
 */

#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

/*
 * Base address of all AHB1 peripheral
 */

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASE + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)


/*
 * Base address of all APB1 peripheral
 */

#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define USART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define USART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)

/*
 * Base address of all APB2 peripheral
 */

#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)

/*
 * Peripheral Register Definition Structure
 */

/* GPIO port mode register (pg. 281)*/
typedef struct{
	__vo uint32_t MODER;   // 0x00
	__vo uint32_t OTYPER; // 0x04
	__vo uint32_t OSPEEDR; // 0x08
	__vo uint32_t PUPDR;// 0x0C
	__vo uint32_t IDR;// 0x10
	__vo uint32_t ODR;// 0x14
	__vo uint32_t BSRR;// 0x18
	__vo uint32_t LCKR;// 0x1C
	__vo uint32_t AFR[2];// 0x20-0x24, GPIO alternate function low and high registers.
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

typedef struct{
	__vo uint32_t IMR; // 0x00
	__vo uint32_t EMR; // 0x04
	__vo uint32_t RTSR; // 0x08
	__vo uint32_t FTSR; // 0x0C
	__vo uint32_t SWIER; // 0x10
	__vo uint32_t PR; // 0x14
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP; // 0x00
	__vo uint32_t PMC; // 0x04
	__vo uint32_t EXTICR[4]; // 0x08 - 0x14
	uint32_t Reserved[2];
	__vo uint32_t CMPCR; // 0x20
}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t CR1; //0x00
	__vo uint32_t CR2; //0x04
	__vo uint32_t SR; // 0x08
	__vo uint32_t DR; //// 0x0C
	__vo uint32_t CRCPR; //0x10
	__vo uint32_t RXCRCR; //0x14
	__vo uint32_t TXCRCR; //0x18
	__vo uint32_t I2SCFGR; //0x1C
	__vo uint32_t I2SPR; //0x20
}SPI_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

typedef struct{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;

/*
 * Peripheral Definitions
 */
#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 					((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2					((USART_RegDef_t*)USART2_BASEADDR)
#define USART6					((USART_RegDef_t*)USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLCK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLCK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLCK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLCK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLCK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLCK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLCK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLCK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLCK_EN() (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLCK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLCK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLCK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLCK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLCK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLCK_EN() (RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLCK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART6_PCLCK_EN() (RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLCK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLCK_DI() (RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLCK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLCK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLCK_DI() (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLCK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLCK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLCK_DI() (RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLCK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLCK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLCK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLCK_DI() (RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLCK_DI() (RCC->APB2ENR &= ~(1 << 14))

/*
 * Macro to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8));  (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

#define SPI1_REG_SET()		do{(RCC->APB2RSTR |= (1 << 12));  (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_SET()		do{(RCC->APB1RSTR |= (1 << 14));  (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_SET()		do{(RCC->APB1RSTR |= (1 << 15));  (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

#define I2C1_REG_SET()		do{(RCC->APB1RSTR |= (1 << 21));  (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_SET()		do{(RCC->APB1RSTR |= (1 << 22));  (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_SET()		do{(RCC->APB1RSTR |= (1 << 23));  (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

#define USART1_REG_SET()	do{(RCC->APB2ENR |= (1 << 4));  (RCC->APB2ENR &= ~(1 << 4)); }while(0)
#define USART2_REG_SET()	do{(RCC->APB1ENR |= (1 << 17)); (RCC->APB1ENR &= ~(1 << 17)); }while(0)
#define USART6_REG_SET()	do{(RCC->APB2ENR |= (1 << 5)); (RCC->APB2ENR &= ~(1 << 5)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)	(	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 : 0)

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLCK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLCK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLCK_EN() (RCC->APB1ENR |= (1 << 15))


#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI5_10		40

/*
 * Priority values for IRQ numbers. Maximum value allowed is 15.
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


/*
 * Some generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/**************************************************************************************
 * 						Bit position definition for SPI peripherals					  *
 **************************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/**************************************************************************************
 * 						Bit position definition for I2C peripherals					  *
 **************************************************************************************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_SWRST		15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/**************************************************************************************
 * 						Bit position definition for USART peripherals					  *
 **************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include "stm32f407xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
