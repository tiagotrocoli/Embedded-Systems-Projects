/*
 * it.c
 *
 *  Created on: Aug 6, 2020
 *      Author: tiago
 */

#include "stm32f4xx_hal.h"

void SysTick_Handler(void){
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
