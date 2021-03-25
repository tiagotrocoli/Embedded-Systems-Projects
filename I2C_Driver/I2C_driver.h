#ifndef __I2C_DRIVER_H
#define __I2C_DRIVER_H

#include "TM4C123.h"                    // Device header

#define ERROR 		1
#define NO_ERROR 	0

// Function prototypes initialize, tranmit and rea functions 
void I2C3_Init ( void );
int I2C3_Write_Multiple(uint8_t slave_address, uint8_t slave_memory_address, uint8_t bytes_count, char* data);
int I2C3_read_Multiple(uint8_t slave_address, uint8_t slave_memory_address, uint8_t bytes_count, char* data);



#endif
