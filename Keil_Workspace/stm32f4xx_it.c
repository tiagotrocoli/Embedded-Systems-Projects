
#include "stm32f407xx.h"

extern void FE_error_callback(void);
extern void DME_error_callback(void);
extern void TE_Complete_callback(void);
extern void HT_Complete_callback(void);
extern void FT_Complete_callback(void);

void EXTI0_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);

void EXTI0_IRQHandler(void){
	
	// send USART2_TX DMA request to DMA1 controller
	USART2->CR3 |= (0x1 << 7);
	// clear the pending flag
	if(EXTI->PR & 0x1){
			EXTI->PR |= (0x1 << 0);
	}
}
		

// IRQ number for DMA1 stream6 global interrupt
void DMA1_Stream6_IRQHandler(void){
	
	// FIFO error interrupt flag
	if(DMA1->HISR & (0x1 << 16)){
		DMA1->HIFCR  |= (0x1 << 16);
		FE_error_callback();
		
		// direct mode error interrupt	flag
	}else if(DMA1->HISR & (0x1 << 18)){
		DMA1->HIFCR  |= (0x1 << 18);
		DME_error_callback();
		
		// transfer error interrupt	flag
	}else if(DMA1->HISR & (0x1 << 19)){
		DMA1->HIFCR  |= (0x1 << 19);
		TE_Complete_callback();
		
		// half-transfer interrupt flag
	}else if(DMA1->HISR & (0x1 << 20)){
		DMA1->HIFCR  |= (0x1 << 20);
		HT_Complete_callback();
		
		// transfer interrupt flag
	}else if(DMA1->HISR & (0x1 << 21)){
		DMA1->HIFCR  |= (0x1 << 21);
		FT_Complete_callback();
		
	}else{
		// nothing
	}
	
}
