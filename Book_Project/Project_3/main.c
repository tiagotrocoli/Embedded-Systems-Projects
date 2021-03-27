#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core

// PD12 --> Green LED
// PD13 --> Orange LED

void task1(void*);
void task2(void*);

TaskHandle_t taskHandle_1 = NULL;
TaskHandle_t taskHandle_2 = NULL;

uint32_t task1_counter = 0;

int main(){
	
	// turn on the clock of registers of GPIOD
	RCC->AHB1ENR = (0x1 << 3);
	// set PD12/13 as output
	GPIOD->MODER = (0x1 << 24) | (0x1 << 26);	
	// turn off green and orange LED
	GPIOD->BSRR = (0x1 << 28) | (0x1 << 29);
	
	xTaskCreate(&task1, "Task_1", configMINIMAL_STACK_SIZE, NULL, 10, &taskHandle_1);
	xTaskCreate(&task2, "Task_2", configMINIMAL_STACK_SIZE, NULL, 10, &taskHandle_2);
	
	vTaskStartScheduler();
	
	while(1);
}

void task2(void *pvParam){
	
	while(1){
		// toggle green LED
		GPIOD->ODR ^= (0x1 << 13);
		// delay for 1s
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

void task1(void *pvParam){
	
	while(1){
		++task1_counter;
		// toggle green LED
		GPIOD->ODR ^= (0x1 << 12);
		// delay for 1s
		vTaskDelay(pdMS_TO_TICKS(1000));
		
		if(task1_counter == 10){
			vTaskSuspend(taskHandle_2);
		}
		
		if(task1_counter == 15){
			vTaskResume(taskHandle_2);
		}
		
	}
}
