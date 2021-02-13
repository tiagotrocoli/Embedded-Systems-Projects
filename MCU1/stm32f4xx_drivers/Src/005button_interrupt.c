/*
 * 005button_interrupt.c
 *
 *  Created on: 8 de jun de 2020
 *      Author: tiago
 */

#include <string.h>
#include "../drivers/Inc/stm32f407xx.h"

void delay(void){
	// ~200ms delay, when system clock is 16Mhz
	for(uint32_t i=0;i<500000/2;i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GpioDtn;

	memset(&GpioLed,0, sizeof(GpioLed));
	memset(&GpioLed,0, sizeof(GpioDtn));

	GpioLed.pGPIOx 								= 	GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	// INPUT = read the state of an electrical signal
	GpioDtn.pGPIOx 								= 	GPIOD;
	GpioDtn.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_5;
	GpioDtn.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_IT_FT;
	GpioDtn.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_OD; because it is an input
	GpioDtn.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioDtn);

	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	while(1);
}

void EXTI9_5_IRQHandler(void){

	delay();
	// clear the pending event from EXTI line
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);

}
