
#include "stm32f4xx.h"                  // Device header

/*
	PC0 - PC7 = D0 - D7
	PB5 = RS (1 = command register, 0 = data register)
	PB6 = R/W (1 = reading, 0 = writing)
	PB7 = EN

*/

#define RS 	0x20
#define RW	0x40
#define EN	0x80

void LCD_Init(void);
void GPIO_Init(void);
void LCD_Command(unsigned char);
void LCD_data(unsigned char);
void delayMs(uint32_t);

int main(void){
	
	
	
	
}

void LCD_Init(void){
	
	
}

void GPIO_Init(void){
	
	// enable the clock of GPIOB and GPIOC
	RCC->AHB1ENR = 0x06;
	
	// define PB5-7 as output pin
	GPIOB->MODER = ((0x1 << 10) | (0x1 << 12) | (0x1 << 14));
	// set EN and RW to LOW (should it be 0xC0 ???)
	GPIOB->BSRR |= (0x0C);
	
	// define PC0-7 as output pin
	GPIOC->MODER = 0x5555;
	
}

void LCD_Command(unsigned char command){
	
	// RS = 0 (set command register) , R\W = 0 (allow writing)
	GPIOB->BSRR = ((RW | RS) << 16);
	// send command to the LCD
	GPIOC->ODR = command;
	// Set EN to HIGH
	GPIOB->BSRR = EN;
	// delay 0 ms
	delayMs(0);
	// Set EN to LOW, so EN goes to HIGH -> LOW to secure command
	GPIOB->BSRR = (EN << 16);
	
}


void LCD_data(unsigned char data){
	// R\W = 0 (allow writing), RS = 1 (set data register) 
	GPIOB->BSRR = ((RW << 16 ) | RS);
	// send data to the LCD
	GPIOC->ODR = data;
	// Set EN to HIGH
	GPIOB->BSRR = EN;
	// delay 0 ms
	delayMs(0);
	// Set EN to LOW, so EN goes to HIGH -> LOW to secure command
	GPIOB->BSRR = (EN << 16);
}


void delayMs(uint32_t delay_in_ms){
	
	
}

