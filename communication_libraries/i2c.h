#ifndef I2C_H
#define I2C_H
#include "stm32f4xx.h"                  // Device header

#define __I2C_CLEAR_ADDRFLAG()    \
  do{                                           \
    __IO uint32_t tmpreg = 0x00U;               \
    tmpreg = I2C1->SR1;       \
    tmpreg = I2C1->SR2;       \
    (void)tmpreg;                             \
  } while(0)

// I2C GPIO pins configuration
void i2c_I2C1_GPIO_config(void);

// I2C peripheral configuration
void i2c_I2C1_config(void);

// I2C check address
int i2c_I2C1_isSlaveAddressExist(uint8_t addrs);

// I2C Transmit (master mode)
int i2c_I2C1_masterTransmit(uint8_t addrs, uint8_t *pData, uint8_t len);

// I2C Receive (master mode)
int i2c_I2C1_masterReceive(uint8_t addrs, uint8_t *pData, uint8_t len);

#endif

