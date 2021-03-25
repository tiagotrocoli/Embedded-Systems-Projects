
#include "TM4C123.h"                    // Device header

#define two_second 2000000

// it generates a delay in microsecond
void Delay_MicroSecond(uint32_t);
// 3% of duty cycle of 50Hz
void Servo_0_Degree(void);
// 7% of duty cycle of 50Hz
void Servo_90_Degree(void);
// 12% of duty cycle of 50Hz
void Servo_180_Degree(void);


int main(){
		
	// configure PA4 as a digitl output
	SYSCTL->RCGCGPIO |= (0x1 << 0);
	// set PA4 as output pin
	GPIOA->DIR |= (0x1 << 4);
	// set PA4 as digital pin
	GPIOA->DEN |= (0x1 << 4);
	
	while(1){
		
		Servo_0_Degree();
		Delay_MicroSecond(two_second);
		Servo_90_Degree();
		Delay_MicroSecond(two_second);
		Servo_180_Degree();
		Delay_MicroSecond(two_second);
	}
	
}

void Servo_0_Degree(void){
	
	for(uint32_t i = 0; i < 50 ;i++){
		
		GPIOA->DATA |= (0x1 << 4);
		Delay_MicroSecond(600);
		GPIOA->DATA &= ~(0x1U << 4);
		Delay_MicroSecond(14000);
		
	}
	
}

void Servo_90_Degree(void){
	
	for(uint32_t i = 0; i < 50 ;i++){
		
		GPIOA->DATA |= (0x1 << 4);
		Delay_MicroSecond(1400);
		GPIOA->DATA &= ~(0x1U << 4);
		Delay_MicroSecond(18600);
		
	}
	
}

void Servo_180_Degree(void){
	
	for(uint32_t i = 0; i < 50 ;i++){
		
		GPIOA->DATA |= (0x1 << 4);
		Delay_MicroSecond(2400);
		GPIOA->DATA &= ~(0x1U << 4);
		Delay_MicroSecond(17600);
		
	}
	
}

// time is in microsecond
void Delay_MicroSecond(uint32_t time){
	
	// enable clock for timer1
	SYSCTL->RCGCTIMER |= (0x1 << 1);
	
	// disable timer before configurig it
	TIMER1->CTL &= ~(0x1U << 0);
	// 16/32-bit timer
	TIMER1->CFG = 0x04;
	// set periodic timer and count down timer
	TIMER1->TAMR |= (0x2 << 0);
	// set the timer to be in micoseconds
	TIMER1->TAILR = 16 - 1;
	// clear time-out flag
	TIMER1->ICR |= (0x1 << 0);
	// enable timer 1
	TIMER1->CTL |= (0x1 << 0);
	
	for(uint32_t i=0; i < time;i++){
		// wait for one microsecond
		while( (TIMER1->RIS & 0x1) == 0);
		// clear time-out flag
		TIMER1->ICR |= (0x1 << 0);
	}
	
}
