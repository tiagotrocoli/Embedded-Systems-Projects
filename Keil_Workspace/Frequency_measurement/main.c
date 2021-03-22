
#include "TM4C123GH6PM.h"                   // Device header
#include <stdio.h>

void Timer0ACapture_init(void);
uint32_t Measure_TimerPeriod(void);
void Delay(unsigned long counter);
void UART5_init(void);
void UART5_Transmitter(unsigned char data);
void printstring(char *str);

int main(){
	
	char mesg[30];
	float period_in_ms = 0, frequency  = 0;
	
	// initialize timer
	Timer0ACapture_init();
	// initialize UART
	UART5_init();
		
	
	while(1){
		// get the period
		period_in_ms = (float) Measure_TimerPeriod();
		// convert into miliseconds
		period_in_ms = (period_in_ms * 62.5) / 1000000.0;
		// get the frequency
		frequency = 1000.0 / period_in_ms;

		/* convert float to string */
		sprintf(mesg, "\r\nFrequency = %f Hz\n", frequency);
		/* print frequency on serial monitor*/
		printstring(mesg);
		Delay(1000);
	}
	
}

void Timer0ACapture_init(void){
	
	// enable clock for PB6
	SYSCTL->RCGCGPIO |= (0x1 << 1);
	// PB6 is a digital pin
	GPIOB->DEN |= (0x1 << 6);
	// PB6 is input
	GPIOB->DIR &= ~(0x1U << 6);
	// enable alternative function
	GPIOB->AFSEL |= (0x1 << 6);
	// define alternative function
	GPIOB->PCTL &= ~(0xFU << 24);
	GPIOB->PCTL |= (0x7 << 24);
	
	// enable clock for TIMER0
	SYSCTL->RCGCTIMER |= (0x1 << 0);
	// disable TIMER0
	TIMER0->CTL &= ~(0x1U);
	// 16bit TIMER
	TIMER0->CFG = 0x4;
	// Capture mode, edge-time mode, timer counts up for TIMERA
	TIMER0->TAMR |= (0x3 << 0) | (0x1 << 2) | (0x1 << 4);
	// Capture positive (rising_ edge mode
	TIMER0->CTL &= ~(0x3U << 2);
	// enable TIMER0
	TIMER0->CTL |= 0x1;
}

uint32_t Measure_TimerPeriod(void){
	
	uint16_t timer1 = 0, timer2 = 0;
	
	// clear flag of edge-time interrupt
	TIMER0->ICR |= (0x1 << 2);
	// wait until the first rising edge is detected
	while( (TIMER0->RIS & 0x4) == 0 );
	// save the timestamp
	timer1 = (uint16_t) TIMER0->TAR ;
	
	// clear flag of edge-time interrupt
	TIMER0->ICR |= (0x1 << 2);
	// wait until the second rising edge is detected
	while( (TIMER0->RIS & 0x4) == 0 );
	// save the timestamp
	timer2 = (uint16_t) TIMER0->TAR ;
	
	// return the period
	return timer2 - timer1;
}

void UART5_init(void)
{
	  SYSCTL->RCGCUART |= 0x20;  /* enable clock to UART5 */
    SYSCTL->RCGCGPIO |= 0x10;  /* enable clock to PORTE for PE4/Rx and RE5/Tx */
    Delay(1);
    /* UART0 initialization */
    UART5->CTL = 0;         /* UART5 module disbable */
    UART5->IBRD = 104;      /* for 9600 baud rate, integer = 104 */
    UART5->FBRD = 11;       /* for 9600 baud rate, fractional = 11*/
    UART5->CC = 0;          /*select system clock*/
    UART5->LCRH = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
    UART5->CTL = 0x301;     /* Enable UART5 module, Rx and Tx */

    /* UART5 TX5 and RX5 use PE4 and PE5. Configure them digital and enable alternate function */
    GPIOE->DEN = 0x30;      /* set PE4 and PE5 as digital */
    GPIOE->AFSEL = 0x30;    /* Use PE4,PE5 alternate function */
    GPIOE->AMSEL = 0;    /* Turn off analg function*/
    GPIOE->PCTL = 0x00110000;     /* configure PE4 and PE5 for UART */
}

void UART5_Transmitter(unsigned char data) {
	
    while((UART5->FR & (1<<5)) != 0); /* wait until Tx buffer not full */
    UART5->DR = data;                  /* before giving it another byte */
}

void printstring(char *str){
	
  while(*str)
	{
		UART5_Transmitter(*(str++));
	}
}
void Delay(unsigned long counter){
	
	unsigned long i = 0;
	
	for(i=0; i< counter*1000; i++);
}



