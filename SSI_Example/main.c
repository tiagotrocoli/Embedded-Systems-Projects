
#include "TM4C123.h"                    // Device header

/*

Digital function 2
PD0 = SSI1Clk *
PD1 = SSI1Fss *
PD2 = SSI1Rx
PD3 = SSI1Tx


Digital function 2
PF0 = SSI1Rx *
PF1 = SSI1Tx *
PF2 = SSI1Clk
PF3 = SSI1Fss

*/

void SSIWrite(unsigned char);
void SSI_Init(void);

int main(void){
	
	SSI_Init();
	unsigned char i;
	
	while(1){
		for(i='A';i<'Z';i++){
			SSIWrite(i);
		}		
	}
	
}

void SSI_Init(void){
	
	// configure PD0 (SSI1Clk) and PD3(SSI1Tx)
	SYSCTL->RCGCGPIO |= (0x1 << 3);
	// disable analog function of PD0 and PD3
	GPIOD->AMSEL &= ~(0x09U);
	// set PD0 and PD3 as output
	//GPIOD->DIR |= (0x1 << 0) | (0x1 << 3);
	// set PD0 and PD3 as digital	pin
	GPIOD->DEN |= (0x1 << 0) | (0x1 << 3);
	// set PD0 and PD3 as alternate function
	GPIOD->AFSEL |= (0x1 << 0) | (0x1 << 3);
	// select anternate function as SSI port
	GPIOD->PCTL &= ~((0xFU << 0) | (0xFU << 12));
	GPIOD->PCTL |= (0x2 << 0) | (0x2 << 12);
	
	// configure PF2
	// enable GPIOF register
	SYSCTL->RCGCGPIO |= (0x1 << 5);
	// enable digital PF2
	GPIOF->DEN |= (0x4);
	// set PF2 as output pin
	GPIOF->DIR |= (0x04);
	// write same data at PF2
	GPIOF->DATA |= (0x04);
	
	// configure SSI1
	// enable register of SSI1
	SYSCTL->RCGCSSI |= (0x1 << 1);
	// clear SSE bit
	SSI1->CR1 &= ~(0x1U << 1);
	// Set this device as master
	SSI1->CR1 = 0;
	// set the clock as System clock
	SSI1->CC = 0;
	// set the pre-scale as 2, so the clock is 8Mhz
	SSI1->CPSR = 2;
	// set 8-bit data size
	SSI1->CR0 = 0x07;
	// enable SSI1
	SSI1->CR1 |= (0x1U << 1);
}

void SSIWrite(unsigned char data){
	
	GPIOF->DATA &= ~(0x04U);
	// wait until the transmit FIFO is not full (one of 16 slots must be empty)
	while( (SSI1->SR & 0x2) == 0);
	SSI1->DR = data;
	// wait until SSI is idle
	while( (SSI1->SR & 0x10) == 1);
	GPIOF->DATA |= (0x04U);
	
}

