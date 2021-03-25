
#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

void FE_error_callback(void);
void DME_error_callback(void);
void TE_Complete_callback(void);
void HT_Complete_callback(void);
void FT_Complete_callback(void);

void button_init(void);
void uart2_init(void);
void dma1_init(void);
void send_some_data(void);
void dma1_interrupt_enable(void);

static char data_stream[] = "Hello World\r\n";

int main(void){
	
	button_init();
	uart2_init();
	dma1_init();
	dma1_interrupt_enable();
	
	while(1);
	
}

void button_init(void){
	
	// user button is connected to PA0.
	
	// 1. enable the clock of PA0 peripheral
	RCC->AHB1ENR |= (0x1 << 0);
	// 2. put gpio as input mode
	GPIOA->MODER &= ~(0x3U);
	// 3. enable the interrupt over the gpio PIN
	EXTI->IMR |= (0x1 << 0);
	// 4. enable the clock of SYSCFG 
	RCC->APB2ENR |= (0x1 << 14);
	// 5. configure the SYSCFG CR4 register
	SYSCFG->EXTICR[0] &= ~(0xFU << 0); // clearing
	SYSCFG->EXTICR[0] |= (0x0 << 0); // setting
	// 6. configure the edge detection on that gpio pin (rising edge)
	EXTI->RTSR |= (0x1 << 0);
	// 7. enable the IRQ related to that gpio pin in NVIC of the processor
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void send_some_data(void){
	
	uint32_t dataLen = strlen(data_stream);
	
	for(uint32_t i=0; i < dataLen ;i++){
		// wait until DR register is empty
		while(!(USART2->SR & (1 << 7)));
		// sent a byte of data
		USART2->DR = (uint8_t) data_stream[i];
	}
	
}

void uart2_init(void){
	
	// 1. enable peripheral clock for uart2 peripheral
	RCC->APB1ENR 	|= (0x1 << 17);
	// 2. configure the gpio pins for uart_tx and uart_rx
	
	// 2.1 enable peripheral clock for PA2(TX) and PA3(RX)
	RCC->AHB1ENR 	|= (0x1 << 0);
	
	// 2.2 configure PA2 (TX)
	GPIOA->MODER 	&= ~(0x3U << 4); // clear
	GPIOA->MODER 	|= (0x2 << 4);		// set it as AF
	GPIOA->AFR[0] &= ~(0xFU << 8); // clear
	GPIOA->AFR[0] |= (0x7 << 8);   // set AF7 mode
	
	//2.3 turn on PULL-UP registor for PA2
	GPIOA->PUPDR &= ~(0x2U << 4);
	GPIOA->PUPDR |= (0x1 << 4);
	
	// 2.4 configure PA3 (RX)
	GPIOA->MODER 	&= ~(0x3U << 6); // clear
	GPIOA->MODER 	|= (0x2 << 6); 		// set it as AF
	GPIOA->AFR[0] &= ~(0xFU << 12); // clear
	GPIOA->AFR[0] |= (0x7 << 12);		// set AF7 mode
	
	//2.5 turn on PULL-UP registor for PA3
	GPIOA->PUPDR &= ~(0x2U << 6);
	GPIOA->PUPDR |= (0x1 << 6);
	
	// 3. configure the baudrate
	USART2->BRR = 0x8B;
	
	// 4. configure the data width, no. of stop bits, etc...
	// <it's not required, since it'll use the default values>
	
	// 5. enable the TX engine of the uart peripheral
	USART2->CR1 |= (0x1 << 3);
	// 6. enable the uart2 peripheral
	USART2->CR1 |= (0x1 << 13);
	
}

void dma1_init(void){
	
	// 1. enable the peripheral clock for the dma1
	RCC->AHB1ENR |= (1 << 21);
	// 2. identify the stream which is suitable for the peripheral 
	// stream 6, channel 4
	
	// 3. identify the channel number on which uart2 peripheral send data
	DMA1_Stream6->CR &= ~(0x7U << 25);
	DMA1_Stream6->CR |= (0x4 << 25);
	// 4. program the source address (memory)
	DMA1_Stream6->M0AR = (uint32_t) data_stream;
	// 5. program the destination address
	DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
	// 6. program number of data itens do send
	DMA1_Stream6->NDTR = (uint32_t) strlen(data_stream);
	// 7. the direction of data transfer (m2p, p2m or m2m)
	DMA1_Stream6->CR |= (0x1 << 6);
 	// 8. program the source and destination data width
	DMA1_Stream6->CR &= ~(0x3U << 13);
	DMA1_Stream6->CR &= ~(0x3U << 11);
	//8a . enable memory auto increment 
	DMA1_Stream6->CR |= (0x1 << 10);
	// 9. direct mode or fifo mode
	DMA1_Stream6->FCR |= (0x1 << 2); // disable direct mode
	// 10. select fifo treshold
	DMA1_Stream6->FCR &= ~(0x3U << 0); // full FIFO
	DMA1_Stream6->FCR |= (0x3 << 0); // full FIFO
	// 11. enable the circular mode if required
	// <it is disable by defalut>
	// 12. single transfer or burst transfer
	
	// 13. configure the stream priority
	// <by defalut it is low-priority>
	
	// 14. enable the stream
	DMA1_Stream6->CR |= (0x1 << 0);
	
}

void dma1_interrupt_enable(void){
	
	// 1. enable half-transfer  IE
	DMA1_Stream6->CR |= (0x1 << 3);
	// 2. enable transfer complete IE
	DMA1_Stream6->CR |= (0x1 << 4);
	// 3. enable transfer error IE
	DMA1_Stream6->CR |= (0x1 << 2);
	// 4. FIFO overrun/underrun IE
	DMA1_Stream6->FCR |= (0x1 << 7);
	// 5. direct mode error
	DMA1_Stream6->CR |= (0x1 << 1);
	
	// 6. enable the IRQ for DMA1 stream6 global NVIC
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void FE_error_callback(void){
	
	while(1);
	
}

void DME_error_callback(void){
	
	while(1);
	
}

void TE_Complete_callback(void){

	while(1);
	
}

void HT_Complete_callback(void){

}

void FT_Complete_callback(void){
	
	// program number of data itens do send
	DMA1_Stream6->NDTR = (uint32_t) strlen(data_stream);
	
	// unable the USART2_TX DMA request to DMA1 controller
	USART2->CR3 &= ~(0x1U << 7);
	
	// enable the stream
	DMA1_Stream6->CR |= (0x1 << 0);
	
	// enable EXT0 interrupt by software
	//EXTI->SWIER |= (0x1 << 0);
}


