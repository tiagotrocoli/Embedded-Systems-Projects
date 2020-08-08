/*
 * stm32407xx_gpio_driver.c
 *
 *  Created on: 15 de mai de 2020
 *      Author: tiago
 */

#include "stm32f407xx.h"

/*
 ****************************************************************************
 * 							APIs Supported by this driver					*
 ****************************************************************************
 */


/*
 * 	Peripheral Clock setup
 */

/*****************************************************
 *
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- this function enables and disables peripheral clock for given GPIO port.
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLCK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLCK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLCK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLCK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLCK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLCK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLCK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLCK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLCK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLCK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLCK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLCK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLCK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLCK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLCK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLCK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLCK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLCK_DI();
		}
	}

}

/*
 * Init and De-init
 * store all content from GPIO_PinConfig to pGPIOx
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	// Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else{

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber == GPIO_MODE_IT_RT){
			// 1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber == GPIO_MODE_IT_RFT){
			// 1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		// Enable the clock of SYSCFG registers
		SYSCFG_PCLCK_EN();
		// clear the current value
		SYSCFG->EXTICR[temp1] &= ~(0xF << (temp2 * 4));
		// Store the value
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));

		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. configure the optype
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT){
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	}

	// 5. configure the alt. functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}

}

/*
 * Data read and write
 */
// read output could be 1 or 0 bit
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}

// 16 pin numbers, 16 inputs of 16 bits
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		// wirte 1 to the output register at the bit field corresponding to the pin
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber < 64){
			// program ISER1 register. 32 to 63
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else{
			// program ISER2 register. 64 to 95
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}

	}else{
		if(IRQNumber <= 31){
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber < 64){
			// program ICER1 register. 32 to 63
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else{
			// program ICER2 register. 64 to 95
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*
 * Note: since the priority bits are 4, then only 4 bits number should be implemented. Ex:
 * 7 = 0111, so, 01110000
 * More than 15 is not allowed
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	// don't we have to clear first?
	*(NVIC_PR_BASE_ADDR + 4*iprx) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber){
	// clear the EXTI PR register related to the pin number
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}
