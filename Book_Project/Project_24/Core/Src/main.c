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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGHT_xUARTQueue 1
#define SIZE_xUARTQueue 	30
#define DATA_SIZE 				30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
TaskHandle_t task1 = NULL;
TaskHandle_t task2 = NULL;
TaskHandle_t task3 = NULL;
QueueHandle_t xUARTQueue = NULL;
SemaphoreHandle_t mutexTemp = NULL;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void showTemperature2sec(void*);
static void showTemperature5sec(void*);
static void uartController(void*);
static void init_ADC1(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
	init_ADC1();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  mutexTemp = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  xUARTQueue   = xQueueCreate(LENGHT_xUARTQueue, SIZE_xUARTQueue);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  xTaskCreate(showTemperature2sec, "Show_Temperature_for_2sec", 2*configMINIMAL_STACK_SIZE, NULL, 5, &task1);
	xTaskCreate(uartController, "UART_Controller", 2*configMINIMAL_STACK_SIZE, NULL, 5, &task3);
	xTaskCreate(showTemperature5sec, "Show_Temperature_for_5sec", 2*configMINIMAL_STACK_SIZE, NULL, 5, &task2);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  xPortStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

/* USER CODE BEGIN 4 */

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

/* USER CODE END 4 */

void showTemperature2sec(void *param){
	
	char data[DATA_SIZE];
	double temperature;
	
	while(1){
		
		// Provide the CPU temperature, that is a critical section
		if(xSemaphoreTake(mutexTemp, pdMS_TO_TICKS(1000)) == pdTRUE){
			temperature = getValueFromADC1();
			xSemaphoreGive(mutexTemp);
		}
		
		// create data to send to PC via UART
		sprintf(data, "2 Temperature: %f\r\n", temperature);
		xQueueSend(xUARTQueue, &data, portMAX_DELAY);
		// block task for 2s
		vTaskDelay( pdMS_TO_TICKS(2000) );
	}
	
}

void showTemperature5sec(void *param){
	
	char data[DATA_SIZE];
	double temperature;
	
	while(1){
		
		// Provide the CPU temperature, that is a critical section
		if(xSemaphoreTake(mutexTemp, pdMS_TO_TICKS(1000) == pdTRUE)){
			temperature = getValueFromADC1();
			xSemaphoreGive(mutexTemp);
		}
		
		// create data to send to PC via UART
		sprintf(data, "5 Temperature: %f\r\n", temperature);
		xQueueSend(xUARTQueue, &data, portMAX_DELAY);
		// block task for 5s
		vTaskDelay( pdMS_TO_TICKS(5000) );
	}
	
}

void uartController(void *param){
	
	char data[DATA_SIZE];
	
	while(1){
		
		// wait uartController to receive data from the other two tasks
		if (xQueueReceive(xUARTQueue, &data ,0) == pdTRUE){
			HAL_UART_Transmit(&huart4, (uint8_t *) data, (uint16_t) strlen(data), 2000 );
		}
	}
	
}

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
