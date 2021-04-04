#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core

#include "task.h"                       // ARM.FreeRTOS::RTOS:Core

void error(void);
void task1(void *);
void task2(void *);

TaskHandle_t taskHandle_1 = NULL;
TaskHandle_t taskHandle_2 = NULL;

int main(void){
		
	BaseType_t result1 = xTaskCreate(task1, "Task_1", configMINIMAL_STACK_SIZE, NULL, 10, &taskHandle_1 );
	BaseType_t result2 = xTaskCreate(task2, "Task_2", configMINIMAL_STACK_SIZE, NULL, 10, &taskHandle_2 );
	
	if( (result1 == pdPASS && result2 == pdPASS) ){
		// memory was allocated
		// start the task

		vTaskStartScheduler();
	}else{
		// memory was not allocated
		error();
	}
	
	while(1);
	
}

void task1(void *param){
	
	// turn on the clock of registers of GPIOD
	RCC->AHB1ENR |= (0x1 << 3);
	// set PD12 as output
	GPIOD->MODER |= (0x1 << 24);	
	// turn off green
	GPIOD->BSRR |= (0x1 << 28);
	
	while(1){
		// turn on green LED
		GPIOD->ODR |= (0x1 << 12);
	}
}

void task2(void *param){
	
	// PA0 --> USER button
	// enable clock for register GPIOA 
	RCC->AHB1ENR |= (0x1 << 0);
	// set PA0 as input
	GPIOA->MODER |= (0x0 << 0);
	
	while(1){
		// wait for the USER button to be pressed
		while( (GPIOA->IDR & 0x1) == 0);
		if( uxTaskPriorityGet(taskHandle_1) == 10){
			vTaskPrioritySet(taskHandle_1, 8);
			// turn off green LED
			GPIOD->ODR &= ~(0x1U << 12);
		}
	}
	
}

void error(void){
	
	while(1);
	
}