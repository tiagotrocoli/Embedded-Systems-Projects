// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// That's the address for PB5-PB0, we put value to this addr
#define LIGHT6   (*((volatile unsigned long *)0x400050FC))
#define SENSOR  (*((volatile unsigned long *)0x4002400C))
#define LIGHT2     (*((volatile unsigned long *)0x400253FC))

#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

struct State{
	unsigned long PBOut;
	unsigned long PFOut;
	unsigned long Time;
	unsigned long Next[5];
};
typedef const struct State STyp;

#define goW 	0
#define waitW 1
#define goS	2
#define waitS 3
#define goWalk 4
#define backW 5
#define backS 6

STyp FSM[7]={
	{0x0C,0x02,80,{goW,goW,waitW,waitW,waitW}},
	{0x14,0x02,60,{goS,goS,goS,goWalk,goS}},
	{0x21,0x02,80,{goS,waitS,goS,waitS,waitS}},
	{0x22,0x02,60,{goW,goW,goW,goWalk,goWalk}},
	{0x24,0x08,60,{backW,backW,backS,goWalk,backW}},
	{0x14,0x08,60,{goW,goW,goW,goW,goW}},
	{0x22,0x08,60,{goS,goS,goS,goS,goS}},
};

// ***** 3. Subroutines Section *****

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
// 80 for > 1/2 seconds
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

unsigned long S;  // index to the current state
unsigned long Input; 
int main(void){ 
	volatile unsigned long delay;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	
  SysTick_Init();   // Program 10.2
	  SYSCTL_RCGC2_R |= 0x00000032; // 1) activate clock for Port F, Port B, and Port E
    delay           = SYSCTL_RCGC2_R; // allow time for clock to start
      // Port F
    GPIO_PORTF_LOCK_R  = 0x4C4F434B;  // 2) unlock GPIO Port F
    GPIO_PORTF_CR_R   |= 0x0A;        // allow changes to PF3, PF1
    GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTF_PCTL_R  = 0x00;        // 4) PCTL GPIO on PF3, PF1
    GPIO_PORTF_DIR_R  |= 0x0A;        // 5) PF3, PF1 are outputs
    GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alternate function
    GPIO_PORTF_PUR_R   = 0x00;        // disable pull-up resistor
    GPIO_PORTF_DEN_R  |= 0x0A;        // 7) enable digital I/O on PF3, PF1
    // Port B
    GPIO_PORTB_LOCK_R  = 0x4C4F434B;  // 2) unlock GPIO Port B
    GPIO_PORTB_CR_R   |= 0x3F;        // allow changes to PB5-PB0
    GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTB_PCTL_R  = 0x00;        // 4) PCTL GPIO on PB5-PB0
    GPIO_PORTB_DIR_R  |= 0x3F;        // 5) PB5-PB0 are outputs
    GPIO_PORTB_AFSEL_R = 0x00;        // 6) disable alternate function
    GPIO_PORTB_PUR_R   = 0x00;        // disable pull-up resistor
    GPIO_PORTB_DEN_R  |= 0x3F;        // 7) enable digital I/O on PB5-PB0
    // Port E
    GPIO_PORTE_LOCK_R  = 0x4C4F434B;  // 2) unlock GPIO Port E
    GPIO_PORTE_CR_R   |= 0x07;        // allow changes to PE2-PE0
    GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTE_PCTL_R  = 0x00;        // 4) PCTL GPIO on PE2-PE0
    GPIO_PORTE_DIR_R   = 0x00;        // 5) PE2-PE0 are inputs
    GPIO_PORTE_AFSEL_R = 0x00;        // 6) disable alternate function
    GPIO_PORTE_PUR_R   = 0x00;        // disable pull-up resistor
    GPIO_PORTE_DEN_R  |= 0x07;        // 7) enable digital I/O on PE2-PE0
  
	EnableInterrupts();
	S = goW;
  while(1){
    LIGHT6 = FSM[S].PBOut;  // set lights
		LIGHT2 = FSM[S].PFOut;  // set lights
    SysTick_Wait10ms(FSM[S].Time);
    Input = SENSOR;     // read sensors
    S = FSM[S].Next[Input];
  }
}

