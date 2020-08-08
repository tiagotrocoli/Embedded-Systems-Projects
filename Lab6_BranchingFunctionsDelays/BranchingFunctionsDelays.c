// BranchingFunctionsDelays.c Lab 6
// Runs on LM4F120/TM4C123
// Use simple programming structures in C to 
// toggle an LED while a button is pressed and 
// turn the LED on when the button is released.  
// This lab will use the hardware already built into the LaunchPad.
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// built-in connection: PF0 connected to negative logic momentary switch, SW2
// built-in connection: PF1 connected to red LED
// built-in connection: PF2 connected to blue LED
// built-in connection: PF3 connected to green LED
// built-in connection: PF4 connected to negative logic momentary switch, SW1

#include "TExaS.h"

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

void delay100ms(unsigned long time){
	
	while(time > 0){
		unsigned long i = 1333333;
		while(i>0){
			i=i-1;
		}
		time = time - 1;
	}
}

int main(void){ 
	unsigned long volatile delay;
	unsigned long pf4;
  TExaS_Init(SW_PIN_PF4, LED_PIN_PF2);  // activate grader and set system clock to 80 MHz
  // initialization goes here
	SYSCTL_RCGC2_R |= 0x00000020; // activate clock for port F
	delay = SYSCTL_RCGC2_R; // allow time for clock F
	GPIO_PORTF_AMSEL_R = 0x00; // disable analog
	GPIO_PORTF_PCTL_R = 0x00000000; // 	clear to configure GPIO
	GPIO_PORTF_DIR_R = 0x04; // set direction. 
	GPIO_PORTF_AFSEL_R = 0x00; // disable alternative function on PF4-0.
	GPIO_PORTF_DEN_R = 0x1F; // enable digital I/O on PF4-0.
	GPIO_PORTF_PUR_R = 0x10; // enable pull-up resistor on PF4 (output)
	GPIO_PORTF_DATA_R |= 0x04; // set LED on in PF4 
	
  EnableInterrupts();           // enable interrupts for the grader
  while(1){
    delay100ms(1);
		pf4 = GPIO_PORTF_DATA_R & 0x10;
		
		if(pf4 == 0x00){ //the switch is pressed
			GPIO_PORTF_DATA_R ^= 0x04;
		}
		else{ // the switch is not pressed
			GPIO_PORTF_DATA_R |= 0x04;
		}
  }
}
