
#include "TM4C123.h"                    // Device header

void SPI1_init(void);
void delay_ms(uint32_t);
void SPI1_write(unsigned char);

// PD2 SSI1Rx (2)
// PD3 SSI1Tx (2)
// PD0 SSI1Clk (2)

int main(){
		
	unsigned char val1 = 'A';
	unsigned char val2 = 'B';
	
	SPI1_init();
	
	while(1){
		SPI1_write(val1);
		delay_ms(1000);
		SPI1_write(val2);
		delay_ms(1000);
	}
	
}

void SPI1_write(unsigned char data){
	
	// make SS low (active slave device)
	GPIOF->DATA &= ~(0x1U << 2);
	// wait until transmit FIFO is not FULL
	while((SSI1->SR & 0x2) == 0);	
	SSI1->DR = data;
	// wait until the data is sent
	while( (SSI1->SR & 0x10) == 1);
	// make SS HIGH in idle condition
	GPIOF->DATA |= (0x1U << 2);
	
}

void SPI1_init(void){
	
	/*  Configure pins for SPI1  
			PD2 SSI1Rx (2)
			PD3 SSI1Tx (2) (not configured)
			PD0 SSI1Clk (2)
			PF2 CSS
	*/
	
	// enable clock for GPIOD
	SYSCTL->RCGCGPIO |= (0x1 << 3);
	// disable analog function for PD0 and PD3
	GPIOD->AMSEL &= ~( (0x1U << 0) | (0x1U << 3));
	// set PD0, PD3 as digital pin
	GPIOD->DEN |= ( (0x1 << 0) | (0x1 << 3));
	// set PD0, PD3 as alternate function
	GPIOD->AFSEL |= ( (0x1 << 0) | (0x1 << 3));
	// set PD0, PD3 as SSI1 pin functions
	GPIOD->PCTL &= ~( (0xFU << 0) | (0xFU << 12));
	GPIOD->PCTL  |= ( (0x2 << 0) | (0x2 << 12));
	
	// enable clock for GPIOF
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	/* set PF2 pin digital */
	GPIOF->DEN |= (0x1<<2); 
	/* set PF2 pin output */
  GPIOF->DIR |= (0x1<<2);         
	/* keep SS idle high */
  GPIOF->DATA |= (0x1<<2);        
	
	/* Configure SPI1 */
	
	// enable the clock of SSI1
	SYSCTL->RCGCSSI |= (0x1 << 1);
	// set this device as master mode
	SSI1->CR1 = 0;
	// set system clock (16Mhz)
	SSI1->CC = 0;
	// set pre-scaler as 4, so 16Mhz / 4 = 4Mhz
	SSI1->CPSR = 4;
	// set data size as 8bit, SPI mode
	SSI1->CR0 |= (0x7 << 0);
	// enable SSI1
	SSI1->CR1 |= (0x1 << 1);
	
}

void delay_ms(uint32_t time_ms){
	
		// enable module 0 register
	SYSCTL->RCGCTIMER |= (0x1 << 0);
	// disable TIMER A	
	TIMER0->CTL |= ~(0x1U << 0);
	// select 16bit Timer
	TIMER0->CFG = 0x4;
	// one-shot mode
	TIMER0->TAMR |= (0x2 << 0);
	// TIMER A prescale
	TIMER0->TAPR = 100 - 1;
	// interrupt occurs for every 0.1s
	TIMER0->TAILR = 16000 - 1;
	// clear flag of interrupt time-out
	TIMER0->ICR = 0x1;
	// enable TIMER A	
	TIMER0->CTL |= (0x1 << 0);
	
	for(uint32_t i=0;i<time_ms;i++){
		
		// wait 0.1s
		while((TIMER0->RIS & 0x1) == 0);
		// clear flag of interrupt time-out
		TIMER0->ICR = 0x1;
		
	}
	
}
