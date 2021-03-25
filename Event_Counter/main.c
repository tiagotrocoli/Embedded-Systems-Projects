
#include "TM4C123.h"                    // Device header

void Timer3A_countCapture_init(void);
void config_PB2(void);
uint32_t Timer3A_countCapture(void);

uint32_t number_of_events = 0;

int main(void){
	
	config_PB2();
	Timer3A_countCapture_init();
	
	while(1){
		number_of_events = Timer3A_countCapture();
	}

}

void Timer3A_countCapture_init(void){
	
	// enable TIMER A block 3 register
	SYSCTL->RCGCTIMER |= (0x1 << 3);
	// disable TIMER A	
	TIMER3->CTL &= ~(0x1U << 0);
	// select 16bit Timer
	TIMER3->CFG = 0x4;
	// capture mode
	TIMER3->TAMR |= (0x3 << 0);
	// edge-count mode
	TIMER3->TAMR |= (0x0 << 2);
	// timer count up mode (start with 0x0)
	TIMER3->TAMR |= (0x1 << 4);
	// the count limit is (2^16 - 1)
	TIMER3->TAMATCHR = 0xFFFF;
	// it extends the value of TAMATCHR, making the limit to be 0xFFFFFF
	TIMER3->TAPMR = 0xFF;
	// capture the rising edge (positive edge = rising edge?)
	TIMER3->CTL &= ~(0xCU);
	// enable TIMER A	
	TIMER3->CTL |= (0x1U << 0);
	
}

void config_PB2(void){
	
		// turn on the clock of PB 
	SYSCTL->RCGCGPIO |= (0x1 << 1);
	// set PB2 as input mode
	GPIOB->DIR &= ~(0x1U << 2);
	// set PB2 as digital
	GPIOB->DEN |= (0x1 << 2);
	// set PB2 as alternate function mode 
	GPIOB->AFSEL |= (0x1 << 2);
	// pull-down register
	GPIOB->PDR |= (0x1 << 2);
	// set alternate function T3PCC0
	GPIOB->PCTL &= ~(0xFU << 8); // clear
	GPIOB->PCTL |= (0x7 << 8);   // set
}

uint32_t Timer3A_countCapture(void){
	uint32_t counter = 0;
	// this register contains the number of rising edges
	counter = TIMER3->TAR;
	return TIMER3->TAR;
}
