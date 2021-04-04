/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

#define WAIT		1
#define ALLOW 	0
#define LENGHT_xUARTQueue	1

UART_HandleTypeDef huart4;
TaskHandle_t task1 = NULL;
TaskHandle_t task2 = NULL;
TaskHandle_t task3 = NULL;
TaskHandle_t task4 = NULL;
QueueHandle_t xUARTQueue;

uint8_t flag_uartController = WAIT;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);

void mainTask(void *);
void uartController(void *);
void lcdController(void *);
void buzzerController(void *);


int main(void){
  
	HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_UART4_Init();
 
	xTaskCreate(mainTask, "Main_Task", 2*configMINIMAL_STACK_SIZE, NULL, 5, &task1);
	xTaskCreate(uartController, "uartController", 2*configMINIMAL_STACK_SIZE, NULL, 5, &task2);
	
	xUARTQueue = xQueueCreate(LENGHT_xUARTQueue, sizeof(uint8_t));
	
  xPortStartScheduler();
	
	
  while (1);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}


/*
	ADC1 characteristics: 
		discontinuos(single) convertion mode, 12-bit resolution, channel 16, '5 CPU-cycles time to convert.
*/
void init_ADC1(void){
	
	// enable ADC1 register
	RCC->APB2ENR |= (0x1 << 8);
	// disable ADC1
	ADC1->CR2 = 0;
	// sampling time is 3 CPU cycles
	ADC1->SMPR1 = (0x1 << 18);
	// enable temperature sensor
	ADC->CCR |= (0x1 << 23);
	// channel 16 (the only one) is the first channel to be converted .
	ADC1->SQR3 = 16;
	// enable ADC1
	ADC1->CR2 |= 0x1;
	
}

double getValueFromADC1(void){
	
	double data, voltage, celcius;	
	// start the convertion
	ADC1->CR2 |= (0x1 << 30);
	// wait until convertion completes
	while( (ADC1->SR & 0x2) == 0);
	// get the converted data and clear flag
	data 			= ADC1->DR; // after vtaskDelay, the code stops here...
	voltage = (double) ( (3.3*data) / 4095 );
	celcius   = (voltage - 0.76) / (0.0025) + 25;

	return celcius;
	
}

void send_UART(char data[]){
	HAL_UART_Transmit(&huart4, (uint8_t *) data, (uint16_t) strlen(data), 2000 );
}

void mainTask(void *param){
	
	double celcius;
	char data[30];
	uint8_t SetPoint = 30;
	
	init_ADC1();
	
	while(1){
		// block 
		flag_uartController = WAIT;
		
		// get the CPU temperature and send to PC
		celcius = getValueFromADC1();
		sprintf(data, "\r\nTemperature: %f\n\r", celcius);
		send_UART(data);
		
		// get the new setPoint from uartController
		if (xQueueReceive(xUARTQueue, &SetPoint, pdMS_TO_TICKS(500)) == pdTRUE){
			// send the new setPoint to PC
			SetPoint = SetPoint - '0';
			sprintf(data, "\r\nSetPoint: %d\n\r", SetPoint);
			send_UART(data);
		}
		
		// allow uartController to run
		flag_uartController = ALLOW;
		
		// block for 1s
		vTaskDelay( pdMS_TO_TICKS(1000) );
	}
	
}


void uartController(void *param){
	
	uint8_t SetPoint;
	
	while(1){
		// wait until mainTask allow it to run
		while(flag_uartController == WAIT);
			
		// wait the user to provide temperature setPoint (can change delay to not wait user...)
		send_UART("\r\nEnter Temperature SetPoint (degrees): ");
		HAL_UART_Receive(&huart4, &SetPoint,sizeof(uint8_t),portMAX_DELAY );
		send_UART("\r\nTemperature SetPoint changed...");
		
		// send the new setPoint to mainTask
		xQueueSend(xUARTQueue, &SetPoint, 1000);
	}
	
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
