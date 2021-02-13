/*
 * it.c
 *
 *  Created on: Aug 6, 2020
 *      Author: tiago
 */

#include "main.h"

extern TIM_HandleTypeDef htime6;

void SysTick_Handler(void){
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM6_DAC_IRQHandler(void){

	HAL_TIM_IRQHandler(&htime6);

}
