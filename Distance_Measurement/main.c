
#include <TM4C123GH6PM.h>
#include <string.h>
#include <stdio.h>

void UART5_init(void);
void UART5_Transmitter(char *);
void delay_microseconds(uint32_t);
void configure_delay(void);
void config_output(void);
void config_input(void);
void Timer0ACapture_init(void);
uint32_t Measure_distance(void);
void Delay(unsigned long counter);

int main(void){
	
	/*stores pulse on time */
	uint32_t time;
	uint32_t distance;
	char mesg[20];
	
	UART5_init();
	config_output();
	config_input();
	configure_delay();
	Timer0ACapture_init();
	
	while(1){
		
		time = Measure_distance(); /* take pulse duration measurement */ 
		distance = (time * 10625)/10000000; /* convert pulse duration into distance */
		sprintf(mesg, "\r\nDistance = %d cm", distance); /*convert float type distance data into string */
			UART5_Transmitter(mesg);
		Delay(2000);	
	}
	
}

/* This function captures consecutive rising and falling edges of a periodic signal */
/* from Timer Block 0 Timer A and returns the time difference (the period of the signal). */
uint32_t Measure_distance(void){
	
	uint32_t lastEdge, thisEdge;
	
	/* Given 10us trigger pulse */
	GPIOA->DATA &= ~(1U<<4); /* make trigger  pin high */
	delay_microseconds(10); /*10 seconds delay */
	GPIOA->DATA |= (1<<4); /* make trigger  pin high */
	delay_microseconds(10); /*10 seconds delay */
	GPIOA->DATA &= ~(1U<<4); /* make trigger  pin low */
	
	while(1){
			/* clear timer0A capture flag */
			TIMER0->ICR = 4;
			/* wait till captured */
			while((TIMER0->RIS & 4) == 0);
			/*check if rising edge occurs */
			if(GPIOB->DATA & (1<<6)){
				/* save the timestamp */
				lastEdge = TIMER0->TAR;
				/* detect falling edge */
				/* clear timer0A capture flag */
				TIMER0->ICR = 4;
				/* wait till captured */
				while((TIMER0->RIS & 4) == 0);
				/* save the timestamp */
				thisEdge = TIMER0->TAR;
				/* return the time difference */
				return (thisEdge - lastEdge);
			}		
		}
}

/* Timer0A initialization function */
/* Initialize Timer0A in input-edge time mode with up-count mode */
void Timer0ACapture_init(void){
	
	/* enable clock to Timer Block 0 */
	SYSCTL->RCGCTIMER |= 1;
	
	// disable the TIMER 1
	TIMER0->CTL &= ~1U;
	/* 16-bit option */
	TIMER0->CFG = 0x04;
	// periodic mode
	TIMER0->TAMR |= (0x3 << 0);
	// count-up mode
	TIMER0->TAMR |= (0x1 << 4);
	// set edge-time detection mode
	TIMER0->TAMR |= (0x1 << 2);
	// detects rising and falling edges of PB6
	TIMER0->CTL |= (0x3 << 2);
	// enable the TIMER_A
	TIMER0->CTL |= (0x1 << 0);
	
}

void config_input(void){
	
	// configure PB6 as input T0CCP0 (AF7)
	SYSCTL->RCGCGPIO |= (0x1 << 1);

	// set PB4 as digital input mode
	GPIOB->DIR |= (0x0 << 6);
	GPIOB->DEN |= (0x1 << 6);
	// enable alternate function for PB6
	GPIOB->AFSEL |= (0x1 << 6);
	// set alternate function as T0CCP0 for PB6
	GPIOB->PCTL &= ~(0xFU << 24);
	GPIOB->PCTL |= (0x7 << 24);
	
}

void config_output(void){
	
	// PA4 connects to ultrasonic sensor
	
	// enable GPIOB peripheral
	SYSCTL->RCGCGPIO |= (0x1 << 0);
	// set PA4 as digital output mode
	GPIOA->DIR |= (0x1 << 4);
	GPIOA->DEN |= (0x1 << 4);
}

/* Create 1 microsecond second delay using Timer block 1 and sub timer A */
void configure_delay(void){
	
	// the prescale (TAPR) is 0, so the freq = 16Mhz, thus
	// for 1 microsecond timeout, the register TAILR should be 16 - 1.
	
	// enable TIMER_A registers
	SYSCTL->RCGCTIMER |= (0x1 << 1);
	
	// disable the TIMER 1
	TIMER1->CTL = 0;
	/* 16-bit option */
	TIMER1->CFG = 0x04;
	// periodic mode
	TIMER1->TAMR |= (0x02 << 0);
	// count-down mode
	TIMER1->TAMR |= (0x0 << 4);
	// set the starting count value
	TIMER1->TAILR = 16 - 1;
	// clears the TIME A timeout flag
	TIMER1->ICR = 0x1;
	// enable the TIMER_A
	TIMER1->CTL |= (0x01 << 0);
	
}

void delay_microseconds(uint32_t time){
	
	// wait for time microseconds
	for(uint32_t i=0;i<time;i++){
		// wait for the timeout (when the timer reaches 1 microseconds)
		while((TIMER1->RIS & 0x1) == 0);
		// clear the timeout flag
		TIMER1->ICR = 0x1;		
	}
}

void UART5_init(void){
	
	// enable UART5 registers
	SYSCTL->RCGCUART |= (0x1 << 5);
	
	// disable UART5
	UART5->CTL = 0; 
	// set baud rate as 9600
	UART5->IBRD = 104;
	UART5->FBRD = 11;
	// UART5 clock is the system clock
	UART5->CC = 0;
	// Data length of 8 bits
	UART5->LCRH |= (0x3 << 5);
	// One stop bit
	// <one stop bit is set by default>
	// No parity
	// <parity is disable by default>
	// enable UART5, RX and TX module
	UART5->CTL |=  (0x1 << 0);
	// enable UART5 TX
	UART5->CTL |= (0x1 << 8);
	// enable UART5 RX
	UART5->CTL |= (0x1 << 9);
	
	
	// PE4 is RX and PE5 is TX of UART5
	// All GPIO pins are push-pull as default
	// enable the GPIOE port
	SYSCTL->RCGCGPIO |= (0x1 << 4);
	
	// Set PE4 as digital
	GPIOE->DEN |= (0x1 << 4);
	// Disable analog mode for PE4 (by default it is disable)
	GPIOE->AMSEL |= (0x0 << 4);
	// Set PE4(RX) as alternate function
	GPIOE->AFSEL |= (0x1 << 4);
	// Set AF as UART5 RX
	GPIOE->PCTL |= (0x1 << 16);
	
	// Set PE5 as digital
	GPIOE->DEN |= (0x1 << 5);
	// Disable analog mode for PE5 (by default it is disable)
	GPIOE->AMSEL |= (0x0 << 5);
	// Set PE5 (TX) alternate function
	GPIOE->AFSEL |= (0x1 << 5);
	// Set AF as UART5 TX
	GPIOE->PCTL |= (0x1 << 20);
	
}

void UART5_Transmitter(char *data){
	
	uint32_t len = strlen(data);
	
	for(uint32_t i=0;i<len;i++){
		while((UART5->FR & (1<<5)) != 0);
		UART5->DR = (uint8_t ) data[i];
	}
	
}

void Delay(unsigned long counter){
	unsigned long i = 0;
	
	for(i=0; i< counter*1000; i++);
}
