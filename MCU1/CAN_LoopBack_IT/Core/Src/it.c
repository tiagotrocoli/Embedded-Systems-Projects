/*
 * it.c
 *
 *  Created on: Aug 6, 2020
 *      Author: tiago
 */

#include "stm32f4xx_hal.h"
extern CAN_HandleTypeDef hcan1;

void SysTick_Handler(void){
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void CAN1_TX_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
}
void CAN1_RX0_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
}
void CAN1_RX1_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
}
void CAN1_SCE_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
}
