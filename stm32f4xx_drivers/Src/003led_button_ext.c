/*
 * 003led_button_ext.c
 *
 *  Created on: 28 de mai de 2020
 *      Author: tiago
 */


#include "../drivers/Inc/stm32f407xx.h"
#define BTN_PRESSED	0

void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;

	// Output = drive a signal HIGH or LOW
	GpioLed.pGPIOx 								= 	GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_FAST;
	// GPIOD output has Pull-up and Pull-down resisters.
		// When GPIOD -> HIGH -> signal HIGH, When GPIOD -> LOW -> signal LOW
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_PP;
	// Pull-down or Pull-up resistors are needed for OP (OPEN-DRAIN), instead of PP.
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	// INPUT = read the state of an eletrical signal
	GPIOBtn.pGPIOx 								= 	GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_OD; because it is an input
	// We need to activate pull-up (PU) or pull-down (PD) register to external button.
	// PU activated = default is HIGH
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1){
		// If the switch is open, GPIOB_12 is HIGH (default)
		// If the switch is closed, GIPOB_12 is LOW.
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED ){
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
		}
	}
}
