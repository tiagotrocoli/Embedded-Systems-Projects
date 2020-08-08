/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: 13 de jun de 2020
 *      Author: tiago
 */

#include "../Inc/stm32f407xx.h"
/*
 * 	Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLCK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLCK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLCK_EN();
		}
	}else{
		// To do
	}
}

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	// Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;
	// 1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY mode by default is 0 (Full-duplex).
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode should be seSPI_Initt
		tempreg |= (1 << 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDI mode should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY mode should be set
		tempreg |= (1 << 10);
	}

	// 3. configure clock
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);

	// 4. configure the DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);
	//tempreg |= pSPIHandle->SPIConfig.SPI_SSM << 9;
	// 5. configure the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);
	// 6. configure the CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);

	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_SET();
	}else if(pSPIx == SPI2){
		SPI2_REG_SET();
	}else if(pSPIx == SPI3){
		SPI3_REG_SET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		//1. wait until TXE is set (that is, until the transmit buffer is empty)
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16 bit DFF
			//1. load the data into DR register
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len--;
			Len--;
			(uint16_t*) pTxBuffer++;
		}else{
			//8 bit DFF
			//1. load the data into DR register
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

	while(Len > 0){
		//1. wait until RXNE is set (that is, until the receive buffer is empty)
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16 bit DFF
			//1. read the data from DR register
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*) pRxBuffer++;
		}else{
			//8 bit DFF
			//1. read the data from DR register
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral control API
 */

/*
 * It enable or disable SPI peripheral
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
