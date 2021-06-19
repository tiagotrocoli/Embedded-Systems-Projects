#include "i2c.h"

// https://sites.google.com/site/johnkneenmicrocontrollers/18b-i2c/i2c_stm32f407#linki2cpins

// I2C GPIO pins configuration
void i2c_I2C1_GPIO_config(void){
	
	// enable GPIOB register
	RCC->AHB1ENR |= (0x1 << 1);
		
	// define the alternate function for PB8 (SCL) and PB9 (SDA)
	GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
	GPIOB->AFR[1] |= ((4 << 0) | (4 << 4));
	
	// select alternate function mode for PB8 (SCL) and PB9 (SDA)
	GPIOB->MODER &= ~((0x3U << 16) | (0x3U << 18));
	GPIOB->MODER |= ((2 << 16) | (2 << 18));
	
	// define PB8 and PB9 as open-drain
	GPIOB->OTYPER |= (0x1 << 8) | (0x1 << 9);
	
	// disable Pull-up and Pull-down register
	GPIOB->PUPDR &= ~((0x3U << 16) | (0x3U << 18));	
}

// I2C peripheral configuration
void i2c_I2C1_config(void){
	
	// enable I2C clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// tell I2C register APB1 clock frequency
	// We set 16Mhz
	I2C1->CR2 = 0x0010U;
	// to generate a signal of 100Khz, CRR = 80
	// In Sm mode I2C, t_low = t_high 
	I2C1->CCR = 0x0050;
	// Rise Time
	I2C1->TRISE = (16 + 1);
	// Enable I2C peripheral
	I2C1->CR1 |= I2C_CR1_PE;
}

// I2C check address
int i2c_I2C1_isSlaveAddressExist(uint8_t addrs){
	
	// send start condition
	I2C1->CR1 &= ~(I2C_CR1_POS);
	I2C1->CR1 |= (I2C_CR1_START);
	// wait until the START condition is generated
	while( !(I2C1->SR1 & I2C_SR1_SB) );
	// send slave address
	I2C1->DR = (uint8_t) addrs;
	// wait for ACK
	while( !(I2C1->SR1 & I2C_SR1_ADDR) );
	// generate stop condition
	I2C1->CR1 |= ( I2C_CR1_STOP);
	// clear ADDR flag
	__IO uint32_t tempRd = I2C1->SR1;
	tempRd = I2C1->SR2;
	(void) tempRd;
	// wait until the bus is not busy
	while( (I2C1->SR2 &  I2C_SR2_BUSY ) );
	
	return 1;
}

// I2C Transmit (master mode)
int i2c_I2C1_masterTransmit(uint8_t addrs, uint8_t *pData, uint8_t len){
	
	// wait until the bus is not busy
	while( (I2C1->SR2 & I2C_SR2_BUSY) );
	
	// 1.0 send start condition
	I2C1->CR1 &= ~(I2C_CR1_POS);
	I2C1->CR1 |= (I2C_CR1_START);
	
	// 1.1 wait until the START condition is generated
	while( !(I2C1->SR1 & I2C_SR1_SB) );
	
	// 2.0 send slave address
	I2C1->DR = (uint8_t) addrs;
	// 2.1 wait for ACK
	while( !(I2C1->SR1 & I2C_SR1_ADDR) );
	// 2.2 clear addrs flag
	__IO uint32_t tempRd = I2C1->SR1;
	tempRd = I2C1->SR2;
	(void) tempRd;
	
	// 3.0 send data
	for(int i=0;i<len;i++){
		// 3. wait the data register to be empty
		while( !(I2C1->SR1 & I2C_SR1_TXE) );
		// 3.2 put the data into the data register
		I2C1->DR = pData[i];
		// 3.3 wait until the data is transmitted
		while( !(I2C1->SR1 &  I2C_SR1_BTF ) );
	}
	
	// 4.0 generate stop condition
	I2C1->CR1 |= ( I2C_CR1_STOP);
	return 1;
}

// I2C Receive (master mode)
int i2c_I2C1_masterReceive(uint8_t addrs, uint8_t *pData, uint8_t len){
	
  //Wait for busy flag
  while(I2C1->SR2 & I2C_SR2_BUSY);
  //Clear POS bit
  I2C1->CR1 &= ~I2C_CR1_POS;

  //Enable ACK
  I2C1->CR1 |= I2C_CR1_ACK;
  //Generate start condition
  I2C1->CR1 |= I2C_CR1_START;
  //Wait for SB Flag
  while(!(I2C1->SR1 & I2C_SR1_SB));
  //Wait for Start Condition generation
  while(!(I2C1->SR1 & I2C_SR1_SB));
  //Send Address, with LSB=0 for Transmit
  //Send slave address
  I2C1->DR = addrs | 0x01;
  //Wait for ADDR flag (confirms Ack is received from Slave device)
  while(!(I2C1->SR1 & I2C_SR1_ADDR));
  if(len==0){
    //Clear ADDR flag
    __I2C_CLEAR_ADDRFLAG();
    //Generate Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
  }
  else if(len == 1)
  {
    //Clear ACK
    I2C1->CR1 &= ~I2C_CR1_ACK;
    //Disable interrupts
    __disable_irq();
    //Clear ADDR flag
    __I2C_CLEAR_ADDRFLAG();
    //Generate Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
    //Enable interrupts
    __enable_irq();
  }
  else if(len == 2)
  {
    //Enable POS bit
    I2C1->CR1 |= I2C_CR1_POS;
    //Disable interrupts
    __disable_irq();
    //Clear ADDR flag
    __I2C_CLEAR_ADDRFLAG();
    //Clear ACK
    I2C1->CR1 &= ~I2C_CR1_ACK;
    //Enable interrupts
    __enable_irq();
  }
  else
  {
    //Enable ACK
    I2C1->CR1 |= I2C_CR1_ACK;
    //Clear ADDR flag
    __I2C_CLEAR_ADDRFLAG();
  }

  //Receive data
  uint8_t dataIdx = 0;
  int8_t dataSize = len;
  while(dataSize > 0)
  {
    if(dataSize <= 3U)
    {
      //One-byte
      if(dataSize == 1U)
      {
        //Wait on RXNE
        while(!(I2C1->SR1 & I2C_SR1_RXNE));
        pData[dataIdx] = (uint8_t)I2C1->DR;
        dataIdx++;
        dataSize--;

      }
      //Two-byte
      else if(dataSize == 2U)
      {
        //Wait on BTF
        while(!(I2C1->SR1 & I2C_SR1_BTF));
        //Disable interrupts
        __disable_irq();
        //Generate Stop
        I2C1->CR1 |= I2C_CR1_STOP;
        //Read data from DR
        pData[dataIdx] = (uint8_t)I2C1->DR;
        dataIdx++;
        dataSize--;
        //Enable interrupts
        __enable_irq();
        //Read data from DR
        pData[dataIdx] = (uint8_t)I2C1->DR;
        dataIdx++;
        dataSize--;
      }
      //Last 3 bytes
      else
      {
        //Wait on BTF
        while(!(I2C1->SR1 & I2C_SR1_BTF));
        //Clear ACK
        I2C1->CR1 &= ~I2C_CR1_ACK;
        //Disable interrupts
        __disable_irq();
        //Read data from DR
        pData[dataIdx] = (uint8_t)I2C1->DR;
        dataIdx++;
        dataSize--;
        //Wait on BTF
        while(!(I2C1->SR1 & I2C_SR1_BTF));
        //Generate Stop
        I2C1->CR1 |= I2C_CR1_STOP;
        //Read data from DR
        pData[dataIdx] = (uint8_t)I2C1->DR;
        dataIdx++;
        dataSize--;
        //Enable interrupts
        __enable_irq();
        //Read data from DR
        pData[dataIdx] = (uint8_t)I2C1->DR;
        dataIdx++;
        dataSize--;
      }
    }
    else
    {
      //Wait on RXNE
      while(!(I2C1->SR1 & I2C_SR1_RXNE));
      pData[dataIdx] = (uint8_t)I2C1->DR;
      dataIdx++;
      dataSize--;
      if(I2C1->SR1 & I2C_SR1_BTF)
      {
        pData[dataIdx] = (uint8_t)I2C1->DR;
        dataIdx++;
        dataSize--;
      }
    }
  }
  return 1;
	
	
}
