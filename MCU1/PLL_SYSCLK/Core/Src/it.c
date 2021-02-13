/*
 * it.c
 *
 *  Created on: Aug 6, 2020
 *      Author: tiago
 */

#include "main.h"

extern UART_HandleTypeDef huart2;

void SysTick_Handler(void){
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

