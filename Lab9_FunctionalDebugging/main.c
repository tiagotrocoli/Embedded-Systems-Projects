// ***** 0. Documentation Section *****
// main.c for Lab 9
// Runs on LM4F120/TM4C123
// In this lab we are learning functional debugging by dumping
//   recorded I/O data into a buffer
// January 15, 2016

// Lab 9
//      Jon Valvano and Ramesh Yerraballi

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****



/* 
This Lab9 starter project is the same as C9_Debugging example but 
includes the connections to the Lab9 grader. You will make three changes. 
First, make the LED flash at 10 Hz. In other words, make it turn on for 0.05 seconds, 
and then turn off for 0.05 seconds. 
Second, make the LED flash if either switch SW1 or SW2 is pressed 
(this means flash the LED if either PF4 or PF0 is 0). 
Third, record PortF bits 4,1,0 every time the input changes or the output changes. 
For example, if your system detects a change in either PF4 or PF0 input, 
record PortF bits 4,1,0. If your system causes a change in PF1, record PortF bits 4,1,0. 

If both PF4 and PF0 switch are not pressed, the PF1 output should be low.  
If either PF4 or PF0 switches is pressed, the output toggles at 10 Hz (±10%). 
Information collected in the Data array matches the I/O on PortF.
50 data points are collected only on a change in input or a change in output.
This means no adjacent elements in the array should be equal.

*/


void PortF_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x00FFFFFF;        // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it             
  NVIC_ST_CTRL_R = 0x00000005;          // enable SysTick with core clock
}
unsigned long Led;
void Delay(void){
	unsigned long volatile time;
  time = 80000; // 0.05sec
  while(time){
   time--;
  }
}
// first data point is wrong, the other 49 will be correct
unsigned long Time[50];
// you must leave the Data array defined exactly as it is
unsigned long Data[50];

unsigned long prevSwitchState1;
unsigned long prevSwitchState4;
unsigned long prevLEDState;

int main(void){  unsigned long i,last,now;
  TExaS_Init(SW_PIN_PF40, LED_PIN_PF1);  // activate grader and set system clock to 16 MHz
  PortF_Init();   // initialize PF1 to output
  SysTick_Init(); // initialize SysTick, runs at 16 MHz
  i = 0;          // array index
  last = NVIC_ST_CURRENT_R;
  EnableInterrupts();           // enable interrupts for the grader
	
	
	
  while(1){
		
		if((GPIO_PORTF_DATA_R & 0x11) != 0x11) {
			
			Led = GPIO_PORTF_DATA_R;   // read previous
			Led = Led^0x02;            // toggle red LED
			GPIO_PORTF_DATA_R = Led;   // output 
			
		} else {
			
			GPIO_PORTF_DATA_R &= ~0x02; // turn off LED
			
		}
   
		if(((GPIO_PORTF_DATA_R & 0x01) != prevSwitchState1) ||
			((GPIO_PORTF_DATA_R & 0x04) != prevSwitchState4) ||
			((GPIO_PORTF_DATA_R & 0x02) != prevLEDState)) {
				
			if(i<50){
				now = NVIC_ST_CURRENT_R;
				Time[i] = (last-now)&0x00FFFFFF;  // 24-bit time difference
				Data[i] = GPIO_PORTF_DATA_R&0x13; // record PF1, PF2, PF4
				last = now;
				i++;
			}
			
			prevSwitchState1 = GPIO_PORTF_DATA_R & 0x01;
			prevSwitchState4 = GPIO_PORTF_DATA_R & 0x04;
			prevLEDState = GPIO_PORTF_DATA_R & 0x02;
			
    }
		
    Delay();
  }
}

/*
unsigned long sw1;
unsigned long sw2;
unsigned long ch1;
unsigned long ch2;
unsigned long ch3;
int main(void){  
	unsigned long i,last,now;
  TExaS_Init(SW_PIN_PF40, LED_PIN_PF1);  // activate grader and set system clock to 16 MHz
  PortF_Init();   // initialize PF1 to output
  SysTick_Init(); // initialize SysTick, runs at 16 MHz
  i = 0;          // array index
  last = NVIC_ST_CURRENT_R;
  EnableInterrupts();           // enable interrupts for the grader
	
	ch1 = GPIO_PORTF_DATA_R & 0x01;
	ch2 = GPIO_PORTF_DATA_R & 0x10;
	ch3 = 0x00;
  while(1){
    
		sw1 = GPIO_PORTF_DATA_R & 0x01;
		sw2 = GPIO_PORTF_DATA_R & 0x10;
		if(sw1 == 0x01 && sw2 == 0x10){
			GPIO_PORTF_DATA_R &= ~0x02; //turn off LED
			if((ch3 != 0x02 || sw1 != ch1 || sw2 != ch2) && i<50){
				Data[i] = GPIO_PORTF_DATA_R&0x13; // record PF1, PF0 and PF4
				i++;
			}
		}else{
			Led = GPIO_PORTF_DATA_R;   // read previous
			Led = Led^0x02;            // toggle red LED
			GPIO_PORTF_DATA_R = Led;   // output
			if(i<50){
				now = NVIC_ST_CURRENT_R;
				Time[i] = (last-now)&0x00FFFFFF;  // 24-bit time difference
				Data[i] = GPIO_PORTF_DATA_R&0x13; // record PF1, PF0 and PF4
				last = now;
				i++;
			}
			Delay();
		}
		ch1 = GPIO_PORTF_DATA_R &0x01;
		ch2 = GPIO_PORTF_DATA_R &0x10;
		ch3 = GPIO_PORTF_DATA_R &0x02;
	}
}*/


// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06
