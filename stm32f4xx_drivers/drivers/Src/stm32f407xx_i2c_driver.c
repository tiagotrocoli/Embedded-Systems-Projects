/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 7 de jul de 2020
 *      Author: tiago
 */

#include "stm32f407xx.h"

static void I2C_generateStartCondition(I2C_RegDef_t	*pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t 	*pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t 	*pI2Cx);

static void I2C_generateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *	pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = (SlaveAddr << 1);
	SlaveAddr &= ~(1);	// slave address is + r/w bit 0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *	pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = (SlaveAddr << 1);
	SlaveAddr |= 1;	// slave address is + r/w bit 0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t 	*pI2Cx){
	uint16_t dummyread = pI2Cx->SR1;
	dummyread = pI2Cx->SR2;
	(void) dummyread;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t 	*pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}

/*
void I2C_Init(I2C_Handle_t *pI2CHandle){

	//ack control bit
	uint32_t tempreg = 0;

	//enable the clock for the I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the frequency field of CR2
	// PCLK1 = APB clock frequency value in Mhz
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// program the device own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//standard mode
		// it is frequency domain and not in time domain,
		// so it is PCLK1 / SCL and not SCL / PCLK1 (as shown in thr reference manual)
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else{
		//fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			// it is frequency domain and not in time domain,
			// so it is PCLK1 / SCL and not SCL / PCLK1 (as shown in thr reference manual)
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			// it is frequency domain and not in time domain,
			// so it is PCLK1 / SCL and not SCL / PCLK1 (as shown in the reference manual)
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	// maximum rise time (tr) is found in pg. 48. I2C bus user-manual book
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//standard mode
		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1;
	}else{
		//fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300)/1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}*/

/*
 * enable the i2c peripheral
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLCK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLCK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLCK_EN();
		}
	}else{
		// To do
	}
}



void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_SET();
	}else if(pI2Cx == I2C2){
		I2C2_REG_SET();
	}else if(pI2Cx == I2C3){
		I2C3_REG_SET();
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){

	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Master send data API
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	// 1. generate the start condition
	I2C_generateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	// NOTE: until SB is cleared SCL will stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// 3. send address to the slave with r/w bit set to position 0 (total 8 bits)
	// Note: it will clear SB register (read SB and write in DR)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// 5. clear that ADDR flag according to its software sequence
	// Note:  Until ADDR is cleared SCL will be stretched (pulled LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. send the data until Len becomes 0
	while(Len > 0){

		// wait until the buffer is empty (TXE = 1)
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	// 7. when Len = 0 wait for TXE = 1 and BTF = 1 before generating the STOP condition
	// Note: TXE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	// when BTF = 1, SCL will be stretched (pulled to LOW)
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// SR = Repeated Start
	if(Sr == I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){

	//1.Generate START condition
	I2C_generateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start generation was completed
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3. send the address of the slave with r/nw bit set to R(1) (8bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address is completed by checking ADDR flag in SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// Read a single byte from Slave
	if(Len ==1){

		//1. Disable Ack
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//2.Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//3.wait until RXNE = 1 (when the master receives data)
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

		//4.Generate STOP condition
		if(Sr == I2C_DISABLE_SR ){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//5. Read data in the buffer DR
		*pRxBuffer = (pI2CHandle->pI2Cx->DR & 0xFF);

	}

	//read multiple bytes
	if(Len>1){

		// clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len is zero
		for(uint32_t i = Len; i > 0; i--){

			//wait until RXNE = 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

			// last 2 bytes remaining
			if(i == 2){

				//disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from data register DR into buffer
			*pRxBuffer = (pI2CHandle->pI2Cx->DR && 0xFF);

			// increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == I2C_ACK_ENABLE){
		// enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);

	}else{
		// disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

}

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
