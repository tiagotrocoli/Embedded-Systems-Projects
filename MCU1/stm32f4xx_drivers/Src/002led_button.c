/*
 * 002led_button.c
 *
 *  Created on: 27 de mai de 2020
 *      Author: tiago
 */

#include "../drivers/Inc/stm32f407xx.h"
#define BTN_PRESSED	1

void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;

	// Output = drive a signal HIGH or LOW
	GpioLed.pGPIOx 								= 	GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_FAST;
	// GPIOD output has Pull-up and Pull-down resisters.
	// When GPIOD -> HIGH -> signal HIGH, When GPIOD -> LOW -> signal LOW
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_PP;
	// Pull-down or Pull-up resistors are needed for OP (OPEN-DRAIN), instead of PP.
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	// INPUT = read the state of an eletrical signal
	GPIOBtn.pGPIOx 								= 	GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_OD; because it is an input
	// there is an internal pull-down register in the board button
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1){
		// If the switch is open, GPIOA_0 is LOW (default)
				// If the switch is closed, GIPOA_0 is HIGH
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED ){
			delay();
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}
	}
}
