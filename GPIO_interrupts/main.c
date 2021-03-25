#include "TM4C123.h"                    // Device header


/*
	PF4 --> SW1
	PF0 --> SW2
	PF1 --> LED (RED)

*/

void delayMs(int delay);

int main(void){
	
	// enable GPIOF register
	SYSCTL->RCGCGPIO |= (0x1 << 5);
		
	// configure PF4 and PF0
	
	// unlock ICR register
	GPIOF->LOCK |= 0x4C4F434B;
	// ?? could be wrong
	GPIOF->CR = 0x01;
	GPIOF->LOCK =0;
	
	// set them as input pin
	GPIOF->DIR &= ~((0x0U << 4) | (0x0U << 0));
	// set PF4 and PF0 as digital pin
	GPIOF->DEN |= ((0x1 << 4) | (0x1 << 0));
	// enable pull-up registor 
	GPIOF->PUR |= ((0x1 << 4) | (0x1 << 0));
	
	__disable_irq();
	// make PF4 and PF0 edge sensitive
	GPIOF->IS &= ~((0x1U << 4) | (0x1U << 0));
	// Interrupt generation is controlled by GPIOIEV
	GPIOF->IBE &= ~((0x1U << 4) | (0x1U << 0));
	// set interrupt falling edge of PF4 and PF0
	GPIOF->IEV &= ~((0x1U << 4) | (0x1U << 0));
	// clear interrupt flag
	GPIOF->ICR |= ((0x1 << 4) | (0x1 << 0));
	// The interrupt from the corresponding pin is sent to the interrupt controller (interrupt mask)
	GPIOF->IM  |= ((0x1 << 4) | (0x1 << 0));
	
	// define interrupt priority
	NVIC_SetPriority(GPIOF_IRQn, 5);
	// enable interrupt for GPIOF
	NVIC_EnableIRQ(GPIOF_IRQn);
	
	__enable_irq();
	
	// configure PF1-3
	// set PF1-3 as output pin
	GPIOF->DIR |= ((0x1 << 1) | (0x1 << 2) | (0x1 << 3));
	// set PF1 as digital pin
	GPIOF->DEN |= ((0x1 << 1) | (0x1 << 2) | (0x1 << 3));
	
	
	while(1){}
		
}

void GPIOF_Handler(void){
	
	if(GPIOF->MIS != 0){
		
		// if SW1 is pressed
		if(GPIOF->MIS & 0x10){
			
			for(uint32_t i=0;i<6;i++){
			// toggle green green
			GPIOF->DATA |= (0x08);
			delayMs(10000);
			GPIOF->DATA &= ~(0x08U);
			delayMs(10000);
			}
			GPIOF->ICR |= ((0x1 << 4) | (0x1 << 0));
			
		// if SW2 is pressed
		}else if(GPIOF->MIS & 0x01){
			
			for(uint32_t i=0;i<6;i++){
				// toggle red green
				GPIOF->DATA |= (0x02);
				delayMs(10000);
				GPIOF->DATA &= ~(0x02U);
				delayMs(10000);
			}
			GPIOF->ICR |= ((0x1 << 4) | (0x1 << 0));
			
		}
	}
}

void delayMs(int delay){
	
	for(int i=0;i<delay;i++){
		for(int j=0;j<10000;j++){}
	}
	
}
