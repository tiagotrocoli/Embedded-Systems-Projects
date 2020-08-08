/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: 27 de jun de 2020
 *      Author: tiago
 */


/* SPI2 PINS
 * PB14 -- > MISO
 * PB15 --> MOSI
 * PB13 -- > SCLK
 * PB12 --> NSS
 * ALT function mode: 5
 */

#include "../Inc/stm32f407xx_gpio_driver.h"
#include "../Inc/stm32f4xx_spi_driver.h"
#include <string.h>
#include <stdio.h>


//command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON			1
#define LED_OFF			0

//arduino analog pin
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//arduino led
#define LED_PIN			9

void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

uint8_t SPI_VerifyRespose(uint8_t ackbyte){
	if(ackbyte == 0xF5){
		//ack
		return 1;
	}else{
		//nack
		return 0;
	}
}

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	//generates sclk of 8MHz
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// software slave management enable for NSS pin
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(){

	GPIO_Handle_t GPIOBtn, GpioLed;
	// INPUT = read the state of an electrical signal
	GPIOBtn.pGPIOx 								= 	GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_OD; because it is an input
	// there is an internal pull-down register in the board button
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	// Output = drive a signal HIGH or LOW
	GpioLed.pGPIOx 								= 	GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		=	GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= 	GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed		= 	GPIO_SPEED_FAST;
	// GPIOD output has Pull-up and Pull-down resisters.
	// When GPIOD -> HIGH -> signal HIGH, When GPIOD -> LOW -> signal LOW
	GpioLed.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OP_TYPE_PP; // ATTENTION test OD also....
	// Pull-down or Pull-up resistors are needed for OP (OPEN-DRAIN), instead of PP.
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);
}

int main(void){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	GPIO_ButtonInit();

	// this function is used to initialized the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();
	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*	SSM = 0 --> hardware manegment
	 * (SSM = 0, SSOE = 1) --> NSS output enable
	 *	(SSM = 0, SSOE = 0) --> NSS output disable
	 *	SPE = 1 --> NSS is HIGH
	 *	SPE = 0 --> NSS is LOW
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){

		// wait until the button is pressed
		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		// to avoid debucing
		delay();
		// this makes NSS signal internally high and avoids MODF error
		//SPI_SSIConfig(SPI2, ENABLE);
		//Enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		/****************** 1.COMMAND_LED_CTRL <pin no(1)>	<value(1)> ****************/
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		// send command code, and the slave will send a data. So the RXNE will be "on", because the slave's data.
		SPI_SendData(SPI2, &commandcode, 1);
		// do the dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if (SPI_VerifyRespose(ackbyte)){
			//send arguments
			args[1] = LED_PIN;
			args[2]	= LED_ON;
			SPI_SendData(SPI2, args, 2);
		}

		/****************** 2. CMD_SENSOR_READ	<analog pin number>****************/
		// wait until the button is pressed
		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		// to avoid debucing
		delay();
		commandcode = COMMAND_SENSOR_READ;
		// send command code, and the slave will send a data. So the RXNE will be "on", because the slave's data.
		SPI_SendData(SPI2, &commandcode, 1);
		// do the dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);
		// read ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if (SPI_VerifyRespose(ackbyte)){
			//send arguments
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);
			// do the dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			//insert some delay so that slave  can ready with the data
			delay();
			// send dummy bits (1 byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);
			// read ack byte received
			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		//let's confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );
		//Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
