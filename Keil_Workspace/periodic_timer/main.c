#include "TM4C123.h"                    // Device header

void timer_periodic(uint32_t time);


int main(void){
		
	// turn on the clock of PF 
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	// set PF3 as output mode
	GPIOF->DIR |= (0x1 << 3);
	// set PF3 as digital
	GPIOF->DEN |= (0x1 << 3);
	
	while(1){
		// wait 10 seconds
		timer_periodic(100);
		GPIOF->DATA ^= (0x1 << 3);
	}
}


void timer_periodic(uint32_t time){
	
	// enable module 0 register
	SYSCTL->RCGCTIMER |= (0x1 << 0);
	// disable TIMER A	
	TIMER0->CTL |= ~(0x1U << 0);
	// select 16bit Timer
	TIMER0->CFG = 0x4;
	// one-shot mode
	TIMER0->TAMR |= (0x2 << 0);
	// TIMER A prescale
	TIMER0->TAPR = 100 - 1;
	// interrupt occurs for every 1s
	TIMER0->TAILR = 16000 - 1;
	// clear flag of interrupt time-out
	TIMER0->ICR = 0x1;
	// enable TIMER A	
	TIMER0->CTL |= (0x1 << 0);
	
	for(uint32_t i=0;i< time;i++){
		
		// wait 0.1s
		while((TIMER0->RIS & 0x1) == 0);
		// clear flag of interrupt time-out
		TIMER0->ICR = 0x1;
		
	}
	
}
