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
#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>

void SystemClockConfig(void);
void UART2_Init(void);
void Error_handler(void);
uint8_t convert_to_capital(uint8_t);

UART_HandleTypeDef huart2;

char *user_data = "The application is running\r";

/*
 * main.c calls HAL_Init(), then it calls HAL_MspInit() (in msp.c)
 * main.c calls UART2_Init()-->HAL_UART_Init(), then it calls HAL_UART_MspInit() (in msp.c)
 */

int main(void){

	HAL_Init();
	SystemClockConfig();
	UART2_Init();

	if(HAL_UART_Transmit(&huart2, (uint8_t*) user_data, (uint16_t) strlen(user_data), HAL_MAX_DELAY) != HAL_OK){
		Error_handler();
	}

	uint8_t rcvd_data;
	uint8_t data_buffer[100];
	uint32_t count = 0;

	while(1){
		HAL_UART_Receive(&huart2, &rcvd_data, 1, HAL_MAX_DELAY);
		if(rcvd_data == '\r'){
			break;
		}else{
			data_buffer[count++] = convert_to_capital(rcvd_data);
		}
	}

	data_buffer[count++] = '\r';
	HAL_UART_Transmit(&huart2, data_buffer, count, HAL_MAX_DELAY);

	while(1);

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

void Error_handler(void){

	while(1);
}





