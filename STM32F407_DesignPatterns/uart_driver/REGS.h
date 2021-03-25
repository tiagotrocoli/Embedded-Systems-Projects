#ifndef __REGS_H
#define	__REGS_H

#include <stdint.h>

// PERIPH_BASE 0x4000 0000
// AHB1 OFFSET 0x0002 0000

#define PERIPH_BASE			0x40000000U
#define AHB1PERIPH_BASE	(PERIPH_BASE + 0x00020000U)
#define RCC_BASE				(AHB1PERIPH_BASE + 0x3800U)

#define GPIOA_BASE			(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE			(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE			(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE			(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE			(AHB1PERIPH_BASE + 0x1000U)
#define GPIOH_BASE			(AHB1PERIPH_BASE + 0x1C00U)

// Create RCC_AHB1ENR register
#define RCC_AHB1ENR			(*(volatile unsigned int *) (RCC_BASE + 0x30))

typedef struct{	
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_TypeDef;

#define	GPIOA						((GPIO_TypeDef *) GPIOA_BASE)
#define	GPIOB						((GPIO_TypeDef *) GPIOB_BASE)
#define	GPIOC						((GPIO_TypeDef *) GPIOC_BASE)
#define	GPIOD						((GPIO_TypeDef *) GPIOD_BASE)
#define	GPIOE						((GPIO_TypeDef *) GPIOE_BASE)
#define	GPIOH						((GPIO_TypeDef *) GPIOH_BASE)

#define GPIOA_EN				(1 << 0)
#define GPIOB_EN				(1 << 1)
#define GPIOC_EN				(1 << 2)
#define GPIOD_EN				(1 << 3)
#define GPIOE_EN				(1 << 4)
#define GPIOH_EN				(1 << 7)

typedef uint32_t Pin_Type;

#endif
