
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
void delayMs1(uint32_t);

int main(void){
	
	LCD_Init();
	
	while(1){
		LCD_data('H');
		delayMs(1);
		LCD_data('E');
		delayMs(1);
		LCD_data('L');
		delayMs(1);
		LCD_data('L');
		delayMs(1);
		LCD_data('O');
		// wait 0.5s
		delayMs(500);
		// clear the screen
		LCD_Command(0x1);
	}
	
}

void LCD_Init(void){
	
	GPIO_Init();
	delayMs(30);
	LCD_Command(0x30);
	delayMs(10);
	LCD_Command(0x30);
	delayMs(1);
	LCD_Command(0x30);
	
	// set 8-bit data mode, 2-line, 5x7 font
	LCD_Command(0x38);
	// move cursor right
	LCD_Command(0x06);
	// clear screen, move cursor home
	LCD_Command(0x01);
	// turn display, blink cursor
	LCD_Command(0x0F);
	
}

void GPIO_Init(void){
	
	// enable the clock of GPIOB and GPIOC
	RCC->AHB1ENR |= 0x06;
	
	// define PB5-7 as output pin
	GPIOB->MODER |= 0x5400;
	// set EN and RW to LOW (should it be 0xC0 ???)
	GPIOB->BSRR = (0x00C);
	
	// define PC0-7 as output pin
	GPIOC->MODER |= 0x5555;
	
}

void LCD_Command(unsigned char command){
	
	// RS = 0 (set command register) , R\W = 0 (allow writing)
	GPIOB->BSRR = ((RS | RW) << 16);
	// send command to the LCD
	GPIOC->ODR = command;
	// Set EN to HIGH
	GPIOB->BSRR = EN;
	// delay 1 ms
	delayMs(1);
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
	// delay 1 ms
	delayMs(1);
	// Set EN to LOW, so EN goes to HIGH -> LOW to secure command
	GPIOB->BSRR = (EN << 16);
}

// delay in milisecond
void delayMs(uint32_t delay_in_ms){
	
	
	// enable clock for TIM2 registers
	RCC->APB1ENR |= 0x1;
	// delay of 1 miliseconds
	TIM2->ARR = 16000 - 1;
	// counter starts from 0
	TIM2->CNT = 0;
	// enable the TIM6
	TIM2->CR1 = 0x1;
	
	
	for(uint32_t i=0; i<delay_in_ms ;i++){
		// waint 1ms
		while( (TIM2->SR & 0x1) == 0  );
		// clear the flag
		TIM2->SR &= ~(0x1U);
	}
	
}

void delayMs1(uint32_t delay_in_ms){
	
	int i;
	for(;delay_in_ms > 0;delay_in_ms--){
		for(i=0;i<3195;i++){}
	}
	
}

