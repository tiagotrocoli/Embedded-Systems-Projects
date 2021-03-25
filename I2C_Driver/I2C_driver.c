#include "I2C_driver.h"

// I2C intialization and GPIO alternate function configuration
void I2C3_Init( void ){
	
	// enable the clock of I2C3
	SYSCTL->RCGCI2C |= (0x1 << 3);
	// define it as master
	I2C3->MCR = (0x1 << 4);
	// set frequency to be 100Khz
	// TPR = 16Mhz / 2*(4 + 6)*100khz - 1
	I2C3->MTPR = 7;
	
	/* configure PD0 (I2C3SCL) and PD1(I2C3SDA)  */
	// enable the clock for GPIOD
	SYSCTL->RCGCGPIO |= (0x1 << 3);
	// enable digital function of PD0-1
	GPIOD->DEN |= 0x3;
	//set PD0-1 as an alternate function
	GPIOD->AFSEL |= 0x3;
	//define PD0-1 as I2C GPIO
	GPIOD->PCTL &= ~(0xFFU);
	GPIOD->PCTL |= (0x33);
	// enaable PD1(I2C3SDA) as open-drain
	GPIOD->ODR |= (0x1 << 1);
	
}

static int I2C_wait_till_done(void){
	
	// wait until the controller is idle
	while(I2C3->MCS & 0x1);
	// return an error if 1, 0 otherwise
	return I2C3->MCS & 0xE;
	
}

int I2C3_Write_Multiple(uint8_t slave_address, uint8_t slave_memory_address, uint8_t bytes_count, char* data){
	
	int i, result;
	
	/* Send slave address that the master wants to communicate (transmit)   */
	// slave address(7 bits), and transmit operation (1st bit)
	I2C3->MSA = (uint8_t) (slave_address << 1);
	//send slave address
	I2C3->MDR = slave_memory_address;
	// START condition followed by TRANSMIT (master goes to the Master Transmit state).
	I2C3->MCS = 0x3;
	// wait until the transmition is completed
	result = I2C_wait_till_done();
	// if an error occurs, stop the transmition
	if(result == ERROR) return ERROR;
	
	// send data, except the last one
	for(i=0;i<bytes_count-1;i++){
		// data to be transmitted
		I2C3->MDR = (uint8_t) *(data + i);
		// TRANSMIT operation (master remains in Master Transmit state).
		I2C3->MCS = 0x1;
		// wait until the transmition is completed
		result = I2C_wait_till_done();
		// if an error occurs, stop the transmition
		if(result == ERROR) return ERROR;
	}
	
	// send the last data and STOP
	I2C3->MDR = (uint8_t) *(data + i);
	// TRANSMIT followed by STOP condition (master goes to Idle state).
	I2C3->MCS = 0x5;
	// wait until the transmition is completed
	result = I2C_wait_till_done();
	// wait until bus is not busy
	while(I2C3->MCS & 0x40);
	// if an error occurs, stop the transmition
	if(result == ERROR) return ERROR;
	
	return NO_ERROR;
	
}

int I2C3_read_Multiple(uint8_t slave_address, uint8_t slave_memory_address, uint8_t bytes_count, char* data){
	
	int i = 0, result;
	
	/* Send slave address that the master wants to communicate (receive)   */
	// slave address(7 bits), and transmit operation (1st bit)
	I2C3->MSA = (uint8_t) (slave_address << 1);
	//send slave address
	I2C3->MDR = slave_memory_address;
	// START condition followed by TRANSMIT (master goes to the Master Transmit state).
	I2C3->MCS = 0x3;
	// wait until the transmition is completed
	result = I2C_wait_till_done();
	// if an error occurs, stop the transmition
	if(result == ERROR) return ERROR;
	
	
	/*  Change the operation to receiver mode */
	// slave address(7 bits), and receive operation (1st bit)
	I2C3->MSA = (uint8_t) ((slave_address << 1) | 0x1);
	
	if(bytes_count == 1){
		//Repeated START condition followed by RECEIVE and STOP condition (master goes to Idle state).
		I2C3->MCS = 0x7;
	}else{
		// START condition followed by RECEIVE (master goes to the Master Receive state).
		I2C3->MCS = 0xB;
	}
	// wait until the controller is idle
	result = I2C_wait_till_done();
	// if an error occurs, stop the read
	if(result == ERROR) return ERROR;
	// read data
	*(data + i) = (char) I2C3->MDR;
	
	// if single byte read, done 
	if (bytes_count == 1){
		// wait until bus is not busy
    while(I2C3->MCS & 0x40);
		return NO_ERROR;
  }
	
	for(i=1;i<bytes_count-1;i++){
		
		// RECEIVE operation (master remains in Master Receive state).
		I2C3->MCS = 0x9;
		// wait until the read is completed
		result = I2C_wait_till_done();
		// if an error occurs, stop the read
		if(result == ERROR) return ERROR;
		// read data
		*(data + i) = (char) I2C3->MDR;
		
	}
	
	// RECEIVE followed by STOP condition (master goes to Idle state).
	I2C3->MCS = 0x5;
	// wait until the controller is idle
	result = I2C_wait_till_done();
	// read data
	*(data + i) = (char) I2C3->MDR;
	// wait until bus is not busy
	while(I2C3->MCS & 0x40);
	
	return NO_ERROR;
}
