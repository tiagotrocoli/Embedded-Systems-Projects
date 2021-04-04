#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core

#include "task.h"                       // ARM.FreeRTOS::RTOS:Core

void error(void);
void task1(void *);

TaskHandle_t taskHandle_1 = NULL;


int main(void){
	
	
	if(xTaskCreate(task1, "Task_1", configMINIMAL_STACK_SIZE, NULL, 10, &taskHandle_1 ) == pdPASS){
		// memory was allocated
		// turn on the clock of registers of GPIOD
		RCC->AHB1ENR = (0x1 << 3);
		// set PD12 as output
		GPIOD->MODER = (0x1 << 24);	
		// turn off green
		GPIOD->BSRR = (0x1 << 28);
		// start the task
		vTaskStartScheduler();
	}else{
		// memory was not allocated
		error();
	}
	
	while(1);
	
}

void task1(void *param){
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t xPeriod = pdMS_TO_TICKS(500);
	
	while(1){
		// toggle green LED
		GPIOD->ODR ^= (0x1 << 12);
		// periodically wake up task for every 500ms
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
	
}

void error(void){
	
	while(1);
	
}
