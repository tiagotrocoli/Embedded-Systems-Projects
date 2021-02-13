/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

#define TRUE 1
#define FALSE 0

void SystemClockConfig(void);
void UART2_Init(void);
void Error_handler(void);
uint8_t convert_to_capital(uint8_t);

UART_HandleTypeDef huart2;
uint8_t data_buffer[100];
uint8_t recvd_data;
uint32_t count = 0;
uint8_t reception_complete = FALSE;

char *user_data = "The application is running\r";

/*
 * main.c calls HAL_Init(), then it calls HAL_MspInit() (in msp.c)
 * main.c calls UART2_Init()-->HAL_UART_Init(), then it calls HAL_UART_MspInit() (in msp.c)
 * UART interrupts: HAL_UART_Receive_IT --> USART2_IRQHandler (it.c) --> HAL_UART_RxCpltCallback
 */

int main(void){

	HAL_Init();
	SystemClockConfig();
	UART2_Init();

	if(HAL_UART_Transmit(&huart2, (uint8_t*) user_data, (uint16_t) strlen(user_data), HAL_MAX_DELAY) != HAL_OK){
		Error_handler();
	}

	// Non-blocking function
	// Note: we could put HAL_UART_Receive_IT inside callback function do avoid being block by while loop
	// and remove while.
	while(reception_complete != TRUE){
		HAL_UART_Receive_IT(&huart2, &recvd_data, 1);
	}

	return 0;

}

uint8_t convert_to_capital(uint8_t data){

	if(data >= 'a' || data <= 'z'){
		data = data - ('a' - 'A');
	}

	return data;
}

void SystemClockConfig(){

}

void UART2_Init(void){
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart2) != HAL_OK){
		// there is a problem
		Error_handler();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(recvd_data = '\r'){
		reception_complete = TRUE;
		data_buffer[count++] = '\r';
		HAL_UART_Transmit(&huart2, data_buffer, count, HAL_MAX_DELAY);
	}else{
		data_buffer[count++] = recvd_data;
	}
}

void Error_handler(void){

	while(1);
}





