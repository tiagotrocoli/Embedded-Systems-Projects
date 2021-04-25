#include "i2c.h"

// I2C GPIO pins configuration
void i2c_I2C1_GPIO_config(void){
	
	// enable GPIOB register
	RCC->AHB1ENR |= (0x1 << 1);
	
	// select alternate function mode for PB8 (SCL) and PB9 (SDA)
	GPIOB->MODER &= ~((0x2U << 16) | (0x2U << 18));
	GPIOB->MODER |= ((0x1 << 16) | (0x1 << 18));
	
	// define the alternate function for PB8 (SCL) and PB9 (SDA)
	GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
	GPIOB->AFR[1] |= ((0x4 << 0) | (0x4 << 4));
	
	// define PB6 and PB7 as open-drain
	GPIOB->OTYPER |= (0x1 << 8) | (0x1 << 9);
	
}

// I2C peripheral configuration
void i2c_I2C1_config(void){
	
	// enable I2C clock
	RCC->APB1ENR |= (0x1 << 21);
	// tell I2C register APB1 clock frequency
	I2C1->CR2 &= ~(1FU);
	// TEM QUE VER QUAL É O CLK!!!!!
	I2C1->CR2 |= 16;
	// Rise Time
	
	// I2C speed (100Mhz)
	
	// Enable I2C peripheral
	I2C1->CR1 |= (0x1);
		
}

// I2C check address
int i2c_I2C1_isSlaveAddressExist(uint8_t addrs, uint8_t timeout);

// I2C Transmit (master mode)
int i2c_I2C1_masterTransmit(uint8_t addrs, uint8_t *pData, uint8_t len, uint32_t timeout);

// I2C Receive (master mode)
int i2c_I2C1_masterReceive(uint8_t addrs, uint8_t *pData, uint8_t len, uint32_t timeout);
