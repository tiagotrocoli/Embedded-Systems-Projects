
#include "TM4C123.h"                    // Device header

// PF2, 5

// create a PMW of 1kHz and 50% of duty cycle

int main(){ 
	
	/* Configure PF2 to output PWM1 channel 6 signal */
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	// PF2 is digital pin
	GPIOF->DEN |= (0x1 << 2);
	// PF2 is PWM1 channel 6 signal
	GPIOF->AFSEL |= (0x1 << 2);
	// PF2 is PWM1 channel 6 signal
	GPIOF->PCTL &= ~(0xFU << 8);
	GPIOF->PCTL |= (0x5 << 8);
	
	
	/* Configure PWM1 channel 6  */
	// enable register of PWM
	SYSCTL->RCGCPWM |= (0x1 << 1);
	// Use system clock without a pre-divider
	SYSCTL->RCC &= ~(0x000100000U);
	// disable PWM1 channel 6
	PWM1->_3_CTL &= ~(0x1U << 0);
	// count-down mode
	PWM1->_3_CTL &= ~(0x1U << 1);
	// if counter = LOAD -> HIGH, when counter = CMPA -> LOW
	PWM1->_3_GENA = 0x0000008C;
	// generates 1kHz
	PWM1->_3_LOAD = 16000 - 1;
	// with 50% of duty cycle
	PWM1->_3_CMPA = 8000 - 1;
	// enable PWM1 channel 6
	PWM1->_3_CTL |= (0x1U << 0);
	// The generated PWM1 channel 6 signal is passed to the pin
	PWM1->ENABLE |= (0x1 << 6);
	
	while(1);
	
}
