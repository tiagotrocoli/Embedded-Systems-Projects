#include "TM4C123.h"                    // Device header
#include "I2C_driver.h"

// PF0 -> SW2 (input)
// PF1 -> RED (output)

int main(){
	
	
	//char data[2] = {0x01,0x02};
	SYSCTL->RCGCGPIO |= 0x20; 
	GPIOF->LOCK = 0x4C4F434B;   // unlockGPIOCR register
	//GPIOF->PUR |= 0x2;        // Enable Pull Up resistor PF1
	GPIOF->DIR |= 0x2;          //set PF1 as an output and PF0 as an input pin
	GPIOF->DEN |= 0x3;         // Enable PF1 and PF4 as a digital GPIO pins 
	
	GPIOF->DATA = 0x2;
}
