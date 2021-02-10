#include "TM4C123.h"                    // Device header

void timer_oneshot(void);


int main(void){
		
	// turn on the clock of PF 
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	// set PF3 as output mode
	GPIOF->DIR |= (0x1 << 3);
	// set PF3 as digital
	GPIOF->DEN |= (0x1 << 3);
	
	//GPIOF->DATA ^= (0x1 << 3);
	
	timer_oneshot();
	
	while(1);
}

// generate 0.1s delay
void timer_oneshot(void){
	
		// enable module 0 register
	SYSCTL->RCGCTIMER |= (0x1 << 0);
	// disable TIMER A	
	TIMER0->CTL |= ~(0x1U << 0);
	// select 16bit Timer
	TIMER0->CFG = 0x4;
	// one-shot mode
	TIMER0->TAMR |= (0x1 << 0);
	// TIMER A prescale
	TIMER0->TAPR = 100 - 1;
	// interrupt occurs for every 1s
	TIMER0->TAILR = 16000 - 1;
	// enable interrupt time-out
	TIMER0->IMR |= (0x1 << 0);
	// clear flag of interrupt time-out
	TIMER0->ICR = 0x1;
	// enable TIMER A	
	TIMER0->CTL |= (0x1 << 0);
	// enable IRQ21
	NVIC->ISER[0] |= (1<<19);
	
}

void TIMER0A_Handler(void){
	
	GPIOF->DATA ^= (0x1 << 3);
	// clear flag of interrupt time-out
	//TIMER0->ICR = 0x1;
}


