
#include "TM4C123.h"                    // Device header


volatile uint32_t temperature;

int main(void){
		
	// enable clock for ADC0
	SYSCTL->RCGCADC |= (0x1 << 0);
	// disable SS3
	ADC0->ACTSS &= ~(0x1U << 3);
	// software trigger convention (The trigger by setting the SSn bit in the ADCPSSI register)
	ADC0->EMUX &= ~(0xFU << 12);
	ADC0->EMUX |= (0x5 << 12);
	// get input from channel 0
	ADC0->SSMUX3 = 0;
	// take temperature sensor sample, set flag at 1st sample
	ADC0->SSCTL3 = 0xE;
	// enable SS3
	ADC0->ACTSS |= (0x1U << 3);
	
	// enable register for WTIMER 0
	SYSCTL->RCGCWTIMER |= 0x1;
	
	// disable WTIMER A
	WTIMER0->CTL = 0;
	// enable 32-bit option
	WTIMER0->CFG = 0x4;
	// enable the output Timer A ADC trigger
	WTIMER0->CTL |= (0x1 << 5);
	// periodic timer mode and down count
	WTIMER0->TAMR |= (0x2 << 0);
	// timer triggers for every 1s 
	WTIMER0->TAILR = 16000000 - 1;
	// enable WTIMER A
	WTIMER0->CTL |= (0x1 << 0);
	
	while(1){
		// wait until complete the convertion
		while( (ADC0->RIS & 0x8) == 0);
		// get the temperature
		temperature =  147 - (247 * ADC0->SSFIFO3) / 4096;
		// clear the flag register in RIS
		ADC0->ISC |= (0x1 << 3);
	}
	
}
