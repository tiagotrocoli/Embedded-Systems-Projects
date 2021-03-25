
#include "TM4C123.h"                    // Device header

#define GREEN (1U << 3)
#define RED		(1U << 2)
#define BLUE	(1U << 4)

void TIMER1A_1Hz_init(void);
void TIMER2A_10Hz_init(void);

int main(){
	
	// setup ports for LED
	SYSCTL->RCGCGPIO |= 0x20;
	GPIOF->DIR = 0x0E;
	GPIOF->DEN =0x0E;
		
	// setup timer1A interrupt
	TIMER1A_1Hz_init();
	// setupt timer2A interrupt
	TIMER2A_10Hz_init();
	
	while(1){
		// toggle BLUE
		//GPIOF->DATA ^= BLUE;
	}
	
}

void TIMER1A_1Hz_init(void){
	
	// enable TIMER 1 register
	SYSCTL->RCGCTIMER |= (0x1 << 1);
	
	// disable Timer A before configuring it
	TIMER1->CTL = 0;
	// 16bit Timer
	TIMER1->CFG = 0x4;
	// periodic timer
	TIMER1->TAMR = 0x02;
	// set 1Hz TIMER
	TIMER1->TAPR = 250;
	TIMER1->TAILR = 64000;
	// enable timer-out interrupt
	TIMER1->IMR |= (1 << 0x0);
	// clear time-out flag
	TIMER1->ICR = (0x1 << 0);
	// enable TIMER A
	TIMER1->CTL |= 0x1;
	// enable interrupt
	NVIC_EnableIRQ(TIMER1A_IRQn);
	//NVIC_SetPriority(TIMER1A_IRQn, 5);
}

void TIMER2A_10Hz_init(void){
	
	// enable TIMER 2 register
	SYSCTL->RCGCTIMER |= (0x1 << 2);
	
	// disable Timer A before configuring it
	TIMER2->CTL = 0;
	// 16bit Timer
	TIMER2->CFG = 0x4;
	// periodic timer
	TIMER2->TAMR = 0x02;
	// set 10Hz TIMER
	TIMER2->TAPR = 250;
	TIMER2->TAILR = 6400;
	// enable timer-out interrupt
	TIMER2->IMR |= (1 << 0x0);
	// clear time-out flag
	TIMER2->ICR |= (0x1 << 0);
	// enable TIMER A
	TIMER2->CTL |= 0x1;
	// enable interrupt
	NVIC_EnableIRQ(TIMER2A_IRQn);
	//NVIC_SetPriority(TIMER1A_IRQn, 5);
	
}

// runs every 1s
void TIMER1A_Handler(void){
	
	// if timer 1 happens
	if(TIMER1->MIS & 0x1){
		
		// toggle RED LED
		GPIOF->DATA ^= RED;
		// clear interrupt flag
		TIMER1->ICR |= (0x1 << 0);
		
	// if timer 2 happens
	}
	
	else{
		// clear everything
		TIMER1->ICR = TIMER1->MIS;
	}
	
}

// runs every 0.1s
void TIMER2A_Handler(void){
	
	if(TIMER2->MIS & 0x1){
		
		// toggle GREEN LED
		GPIOF->DATA ^= GREEN;
		// clear interrupt flag
		TIMER2->ICR |= (0x1 << 0);
	}
	
	else{
		// clear everything
		TIMER2->ICR = TIMER1->MIS;
	}
	
}
