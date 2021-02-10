
#include <TM4C123GH6PM.h>

int main(void){
	
	// configure PF2 (blue LED)
	// enable clock for GPIOF
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	// set PF2 as output
	GPIOF->DIR |= (0x1 << 2);
	// set PF2 as digital pin
	GPIOF->DEN |= (0x1 << 2);
	
	// sysTick uses System Clock (16Mhz)
	SysTick->CTRL |= (0x1 << 2);
	// enable a SysTick interrupt
	SysTick->CTRL |= (0x1 << 1);
	// set a delay of 1s
	// since System clock is 16Mhz, LOAD must be 16*10^6
	SysTick->LOAD = 16000000 - 1;
	//initialize current value register
	SysTick->VAL  = 0;
	// enable the sysTick
	SysTick->CTRL |= (0x1 << 0);
	
	while(1);
	
}

// toggle LED for every 1s
void SysTick_Handler(void){
	
	GPIOF->DATA ^= (0x1 << 2);
	
}