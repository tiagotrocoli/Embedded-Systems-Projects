#include "TM4C123.h"                    // Device header

void Systick_init(void);

int main(void){
		
	// turn on the clock of PF 
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	// set PF3 as output mode
	GPIOF->DIR |= (0x1 << 3);
	// set PF3 as digital
	GPIOF->DEN |= (0x1 << 3);
	
	Systick_init();
	
	while(1);
	
}

void Systick_init(void){
		
	// clock source = System clock
	SysTick->CTRL |= (0x1 << 2);
	// enable interrupt
	SysTick->CTRL |= (0x1 << 1);
	// Systick generates an interrupt for every 1s, so LOAD = 16*10^6
	SysTick->LOAD |= 16000000 - 1;
	// enable the SyStick
	SysTick->CTRL |= (0x1 << 0);
	
}

// toggle green light for every 1s
void SysTick_Handler(void){
	
	GPIOF->DATA ^= (0x1 << 3);
	
}


