

#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core


// PD12 --> Green LED

void task1(void*);

int main(){
	
	// turn on the clock of registers of GPIOD
	RCC->AHB1ENR = (0x1 << 3);
	// set PD12 as output
	GPIOD->MODER = (0x1 << 24);	
	// turn off green LED
	GPIOD->BSRR = (0x1 << 28);
	
	TaskHandle_t taskHandle_1;
	
	xTaskCreate(&task1, "Task_1", configMINIMAL_STACK_SIZE, NULL, 10, &taskHandle_1);
	vTaskStartScheduler();
	
	while(1);
}

void task1(void *pvParam){
	
	while(1){
		// toggle green LED
		GPIOD->ODR ^= (0x1 << 12);
		// delay for 1s
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
