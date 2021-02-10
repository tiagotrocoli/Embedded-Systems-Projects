
#include "TM4C123.h"                    // Device header

void wtimer1A_delaySec(uint32_t time_in_seconds);

int main(){
	
	// enable clock for GPIOF
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	// define PF2 as input
	GPIOF->DIR |= (0x1 << 2);
	// enable digital mode for PF2
	GPIOF->DEN |= (0x1 << 2);
	
	while(1){
		
		GPIOF->DATA ^= (0x1 << 2);
		wtimer1A_delaySec(1);
		
	}
	
}


void wtimer1A_delaySec(uint32_t time_in_seconds){
	
	// enable clock register for WTIMER 1
	SYSCTL->RCGCWTIMER |= (0x1 << 1);
	
	// disable TIMER A
	WTIMER1->CTL = 0;
	// 32-bit mode
	WTIMER1->CFG &= ~(0xFU);
	WTIMER1->CFG |= (0x4);
	// periodic mode
	WTIMER1->TAMR |= 0x2;
	// count-up mode
	WTIMER1->TAMR |= (0x1 << 4);
	// set 1s delay
	WTIMER1->TAILR |= 16000000 - 1;
	// clear the flag bit
	WTIMER1->ICR |= 0x1;
	// enable TIMER A
	WTIMER1->CTL |= (	0x1 << 0);
	
	for(uint32_t i = 0; i < time_in_seconds ;i++){
		// wait 1 second
		while( (WTIMER1->RIS & 0x1) == 0 );
		// clear the flag bit
		WTIMER1->ICR |= 0x1;
	}
	
}

