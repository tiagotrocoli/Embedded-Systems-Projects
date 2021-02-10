
#include "TM4C123.h"                    // Device header

void delay_Microsecond(uint32_t time);
void Timer0_init(void);
void init_PA4_PF3(void);
uint32_t measureD(void);

uint32_t highEdge = 0;
uint32_t lowEdge = 0;
// distance in centimeters
uint32_t distance = 0;

uint32_t distance_in_cm;

/*Value of 1 clock cycle in nanoseconds*/
const double _16MHz_1clock = 62.5e-9;
/*Derived from speed of sound*/
const uint32_t MULTIPLIER  = 5882;

// PB4 is the trigger
// PB6 is the ECCHO
// TIMER0 is the delay
// TIMER1 is to calculate the timer between the rising and falling edge


int main(){
	
	// configure PA4 (trigger) and PF2
	init_PA4_PF3();
	// configure PB6 and TIMER1
	Timer0_init();
	
	while(1){
		distance_in_cm = measureD();
		if(distance_in_cm < 15){
			GPIOF->DATA |= (1 << 2);
		}
		else{
			GPIOF->DATA &= ~(1U << 2);			
		}
		delay_Microsecond(100);
	}

}


uint32_t measureD(void){
	
	// set PA4 as LOW for 10 microsendods
	GPIOA->DATA &= ~(0x1U << 4);
	delay_Microsecond(12);
	// set PA4 as HIGH for 10 microsendods
	GPIOA->DATA = (0x1 << 4);
	delay_Microsecond(12);
	// set PA4 as LOW
	GPIOA->DATA &= ~(0x1U << 4);
	
	// clear the flag
	TIMER0->ICR |= (0x1 << 2);
	// wait until rising edge occurs
	while( (TIMER0->RIS & (0x1 << 2)) == 0 );
	// time for rising edge
	highEdge = TIMER0->TAR;
	
	// clear the flag
	TIMER0->ICR |= (0x1 << 2);
	// wait until falling edge occurs
	while( (TIMER0->RIS & (0x1 << 2)) == 0 );
	// time for falling edge
	lowEdge = TIMER0->TAR;
	distance = lowEdge - highEdge;
	distance = _16MHz_1clock *(double) MULTIPLIER * (double)distance;
	
	return distance;
}

void init_PA4_PF3(void){
	
	// enable GPIOA clock
	SYSCTL->RCGCGPIO |=(1U<<0);
	// set it PB4 as output
	GPIOA->DIR |= (0x1 << 4);
	// set it as digital
	GPIOA->DEN |= (0x1 << 4);
	
	// enable GPIOF clock
	SYSCTL->RCGCGPIO |=(1U<<5);
	// set it PF2 as output
	GPIOF->DIR |= (0x1 << 2);
	// set it as digital
	GPIOF->DEN |= (0x1 << 2);
	
}

void delay_Microsecond(uint32_t time){
	
	// enable the clock of module 1
	SYSCTL->RCGCTIMER |= (0x1 << 1);
	
	// disable TIMER A
	TIMER1->CTL &= ~(0x1U << 0);
	// set it as 16 bit timer
	TIMER1->CFG = 0x4;
	// set it as periodic timer
	TIMER1->TAMR |= (0x2);
	// set delay of 1 microsecond
	TIMER1->TAILR = 16-1;
	// clear time out flag
	TIMER1->ICR |= (0x1 << 0);
	// enable timer A
	TIMER1->CTL |= (0x1U << 0);
	
	// wait time * (1 microsecond)
	for(uint32_t i=0;i<time;i++){
		// wait 1 microsecond
		while( (TIMER1->RIS & 0x1) == 0 );
		// clear the time out flag
		TIMER1->ICR |= (0x1 << 0);
	}
	
}

void Timer0_init(void){
	
	// configure PB6 (0x7)
	
	// enable th clock for GPIOB
	SYSCTL->RCGCGPIO |=(1U<<1);
	
	// set PB6 as input
	GPIOB->DIR &= ~(0x1U << 6);
	// set it as digital pin
	GPIOB->DEN |= (0x1 << 6);
	// enable pull-down register
	GPIOB->PDR |= (0x1 << 6);
	// define PB6 as alternate function
	GPIOB->AFSEL |= (0x1 << 6);
	// define T0PCC0 as alternate function
	GPIOB->PCTL &= ~(0xFU << 24);
	GPIOB->PCTL |= (0x7 << 24);
	
	// enable the clock of module 0
	SYSCTL->RCGCTIMER |= (0x1 << 0);
	
	// disable TIMER A
	TIMER0->CTL &= ~(0x1U << 0);
	// both edge event mode
	TIMER0->CTL |= (0x3 << 2);
	// set it as 16 bit timer
	TIMER0->CFG = 0x4;
	// set it as capture mode
	TIMER0->TAMR |= (0x3);
	// count up timer
	TIMER0->TAMR |= (0x1<<4);
	// edge-time mode
	TIMER0->TAMR |= (0x1 << 2);
	// enable timer A
	TIMER0->CTL |= (0x1U << 0);
	
}

