
#include "TM4C123.h"                    // Device header

// ue ADC0 SS3

uint32_t result;

int main(void){
	
	// enable GPIOE
	SYSCTL->RCGCGPIO |= (0x1 << 4);
	// define PE3 as input
	GPIOE->DIR &= ~(0x0U << 3);
	// enable alternate function
	GPIOE->AFSEL |= (0x1 << 3);
	// enable analog function of PE3
	GPIOE->AMSEL |= (0x1 << 3);
	// disable PE3 digital function
	GPIOE->DEN &= ~(0x1U << 3);
	
	
	
	// enable clock for ADC0
	SYSCTL->RCGCADC |= (0x1 << 0);
	// disable SS3
	ADC0->ACTSS &= ~(0x1U << 3);
	// software trigger convention (The trigger by setting the SSn bit in the ADCPSSI register)
	ADC0->EMUX &= ~(0xFU << 12);
	// get input from channel 0
	ADC0->SSMUX3 = 0;
	// take one sample at a time, set flag at 1st sample
	ADC0->SSCTL3 |=6;
	// enable SS3
	ADC0->ACTSS |= (0x1U << 3);
	
	while(1){
		
		// start a conversion at SS3 ADC0
		ADC0->PSSI |= (0x1 << 3);
		// wait until complete the convertion
		while( (ADC0->RIS & 0x8) == 0);
		
		result = ADC0->SSFIFO3;
		// clear the flag register in RIS
		ADC0->ISC |= (0x1 << 3);
		
	}	
}
