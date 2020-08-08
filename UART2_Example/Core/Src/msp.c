/*
 * msp.c
 *
 *  Created on: Aug 6, 2020
 *      Author: tiago
 */

// #include "../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
#include "stm32f4xx_hal.h"

void HAL_MspInit()
{

  // Here we will do low level processor specific inits.

	// 1. Set up the priority of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// 2. Enable the required system exceptions of the arm cortex mx processor
	SCB->SHCSR |= 0x7 << 16;

	// 3. configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}
