/*
 * main.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "main.h"

void GPIO_Init(void);
void Error_handler(void);
void TIMER2_Init(void);
void UART2_Init(void);
void SystemClock_Config_HSE(uint8_t clock_freq);

TIM_HandleTypeDef htimer2;
UART_HandleTypeDef huart2;
uint32_t ccr_content;

/*
 * Calculation of pulse value (example):
 *
 * APB1_clk = 50*10^(6)Hz
 * APB1_clk = APB1_clk / (pre-escalar(=1) + 1) = 25*10^(6)Hz
 * APB1_clk = 25*10^(6)Hz
 * APB1_period = 4*10^(-8)s
 * desire freq = 500 Hz --> period = 0.002s
 * So, it togles every 0.001s or 1000Hz
 * 0.001s = APB1_period * pulse_value ----> pulse_value = 0.001 / APB1_period = APB1_clk / 1000Hz = 25000
 */

uint32_t pulse1_value = 25000; // produce 500Hz
uint32_t pulse2_value = 12500; // produce 1000Hz
uint32_t pulse3_value = 6250;  // produce 2000Hz
uint32_t pulse4_value = 3125;  // produce 4000Hz

/*
 * Active the PLL clock which with is 50Mhz of HSE.
 *
 */

int main(void){

	HAL_Init();

	/* Active the PLL clock which uses 50Mhz of HSE. */
	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);

	GPIO_Init();
	/* Initialize UART2 that uses GPIOA 2 and 3*/
	UART2_Init();

	/* Initialize TIMER2 that uses GPIOA 0 and 1 and GPIOB 10 and 11 */
	TIMER2_Init();

	/* Activate interrupt mode. */
	if( HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_1) != HAL_OK ){
		Error_handler();
	}
	if( HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_2) != HAL_OK ){
			Error_handler();
	}
	if( HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_3) != HAL_OK ){
			Error_handler();
	}
	if( HAL_TIM_OC_Start_IT(&htimer2, TIM_CHANNEL_4) != HAL_OK ){
			Error_handler();
	}

	/*
	char buffer[30];
	uint32_t sysclk = HAL_RCC_GetSysClockFreq();
	sprintf(buffer, "Sysclk = %lu", sysclk);
	printf("%s", buffer);

	char buffer1[30];
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	sprintf(buffer1, "PCLK1 freq = %lu", pclk1);
	printf("%s", buffer1);*/

	while(1);


	return 0;
}

/**
  * @brief System Clock Configuration
  * FIX system clock!!!!!!
  */
void SystemClock_Config_HSE(uint8_t clock_freq)
{
	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
    uint8_t flash_latency=0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI ;
	Osc_Init.HSEState = RCC_HSE_ON;
	//Osc_Init.LSEState = RCC_LSE_ON;
	Osc_Init.HSIState = RCC_HSI_ON;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clock_freq)
	 {
	  case SYS_CLOCK_FREQ_50_MHZ:
		  Osc_Init.PLL.PLLM = 8;
		  Osc_Init.PLL.PLLN = 100;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;

		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
          flash_latency = 1;
	     break;

	  case SYS_CLOCK_FREQ_84_MHZ:
		  Osc_Init.PLL.PLLM = 8;
		  Osc_Init.PLL.PLLN = 168;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;

		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
          flash_latency = 2;
	     break;

	  case SYS_CLOCK_FREQ_120_MHZ:
		  Osc_Init.PLL.PLLM = 8;
		  Osc_Init.PLL.PLLN = 240;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;

		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
          flash_latency = 3;
	     break;

	  default:
	   return ;
	 }

		if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
	{
			Error_handler();
	}



	if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
	{
		Error_handler();
	}


	/*Configure the systick timer interrupt frequency (for every 1 ms) */
	uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(hclk_freq/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);



 }

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);
}

void UART2_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}


}


 void TIMER2_Init(void){

	 TIM_OC_InitTypeDef tim2OC_init;

	 htimer2.Instance = TIM2;
	 htimer2.Init.Period = 0xFFFFFFFF;
	 // 50Mhz / (Prescaler + 1) =  25Mhz
	 htimer2.Init.Prescaler = 1;

	 if(HAL_TIM_OC_Init(&htimer2) != HAL_OK){
		 Error_handler();
	 }

	 tim2OC_init.OCMode =  TIM_OCMODE_TOGGLE;
	 tim2OC_init.OCPolarity =  TIM_OCPOLARITY_HIGH;
	 tim2OC_init.Pulse = pulse1_value;
	 if( HAL_TIM_OC_ConfigChannel(&htimer2, &tim2OC_init, TIM_CHANNEL_1) != HAL_OK ){
		 Error_handler();
	 }
	 tim2OC_init.Pulse = pulse2_value;
	 if( HAL_TIM_OC_ConfigChannel(&htimer2, &tim2OC_init, TIM_CHANNEL_2) != HAL_OK ){
		 Error_handler();
	 }
	 tim2OC_init.Pulse = pulse3_value;
	 if( HAL_TIM_OC_ConfigChannel(&htimer2, &tim2OC_init, TIM_CHANNEL_3) != HAL_OK ){
		 Error_handler();
	 }
	 tim2OC_init.Pulse = pulse4_value;
	 if( HAL_TIM_OC_ConfigChannel(&htimer2, &tim2OC_init, TIM_CHANNEL_4) != HAL_OK ){
		 Error_handler();
	 }

}


 /*
  * Everytime CNT register == CRRx register, calls HAL_TIM_OC_DelayElapsedCallback
  */
 void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

	 /* It codes doesn't deal with overflow. */

	 /* TIM2_CH1 toggling with frequency = 500Hz */
	 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		 // get the current value in CCR1 register
		 ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		 // update the CCR1
		 __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ccr_content + pulse1_value);
	 }
	 /* TIM2_CH2 toggling with frequency = 1000Hz */
	 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		 // get the current value in CCR2 register
	 	ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	 	// update the CCR2
	 	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ccr_content + pulse2_value);
	 }
	 /* TIM2_CH3 toggling with frequency = 2000Hz */
	 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		 // get the current value in CCR3 register
	 	ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	 	// update the CCR3
	 	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ccr_content + pulse3_value);
	 }
	 /* TIM2_CH4 toggling with frequency = 4000Hz */
	 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		 // get the current value in CCR4 register
	 	ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
	 	// update the CCR4
	 	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, ccr_content + pulse4_value);
	 }

}

void Error_handler(void)
{
	while(1);
}

