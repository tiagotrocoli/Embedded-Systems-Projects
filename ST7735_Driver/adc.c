#include "adc.h"


void ADCI_Init(void)
{
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	
	  ADC_ChannelConfTypeDef sConfig;
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.ScanConvMode = DISABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.DiscontinuousConvMode = ENABLE;
	  hadc1.Init.NbrOfDiscConversion = 1;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 1;

   hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
 	HAL_ADC_Init(&hadc1);
	 
	sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
   HAL_ADC_ConfigChannel(&hadc1, &sConfig);



}
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
	
	    /**ADC1 GPIO Configuration    
    PA4     ------> ADC1_IN4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {

    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA4     ------> ADC1_IN4 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

  }
 

}
