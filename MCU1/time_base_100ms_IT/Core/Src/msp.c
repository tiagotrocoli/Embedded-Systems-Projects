/*
 * msp.c
 *
 *  Created on: Aug 6, 2020
 *      Author: tiago
 */

// #include "../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
#include "main.h"

void HAL_MspInit()
{

	// Low level initialization of processor

	// 1. Set up the priority of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// 2. Enable the required system exceptions of the arm cortex mx processor
	SCB->SHCSR |= 0x7 << 16; // usage fault, memory fault and bus fault system exceptions

	// 3. configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htimer){

	// 1. Enable the clock for TIM6 peripheral
	__HAL_RCC_TIM6_CLK_ENABLE();

	// 2. Enable the IRQ
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	// 3. setup the priority for TIM6_DAC_IRQn
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 15, 0);


}
