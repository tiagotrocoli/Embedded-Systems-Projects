#include "GLCD_ST7735.h"

SPI_HandleTypeDef hspi2;

void ST7735_GPIO_Init(void){
	
	/*SPI Pins
			RES: PB5
			DC : PB4
			CS : PB3	
	*/
	
	__GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin 	= (GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3);
	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void ST7735_SPI_Init(void){
	
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_1LINE;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	
	HAL_SPI_Init(&hspi2);
	
}

void spi2_8b_Init(void){
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	HAL_SPI_Init(&hspi2);
}

void spi2_16b_Init(void){
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	HAL_SPI_Init(&hspi2);
}

// send 8bit data
void lcd7735_send8Data(unsigned char data){
	HAL_SPI_Transmit(&hspi2, &data, 1, 0x1 );
}

// send 16bit data
void lcd7735_send16Data(uint8_t msb, uint8_t lsb){
	uint8_t masData[] = {lsb, msb};
	HAL_SPI_Transmit(&hspi2, masData, 1, 0x1 );
}

void lcd7735_sendData(unsigned char data){
	//set DC HIGH (DC = 1 --> data mode)
	LCD_DC1;
	// send data
	lcd7735_send8Data(data);
}

void lcd7735_sendCommand(unsigned char command){
	//set DC LOW (DC = 0 --> command mode)
	LCD_DC0;
	// send command
	lcd7735_send8Data(command);
}

void standard_Init_Cmd(void){
	
	LCD_CS0;  // CS = 0
	LCD_RST0; // Reset = 0
	HAL_Delay(10); // delay of 10ms
	LCD_RST1; 	// reset = 1
	HAL_Delay(10); // delay of 10ms
	lcd7735_sendCommand(0x11);
	HAL_Delay(120); // delay of 120ms
	
	// set color mode
	lcd7735_sendCommand(0x3A);
	// 16bits
	lcd7735_sendCommand(0x05);
	lcd7735_sendCommand(0x36);
	lcd7735_sendCommand(0x14);
	lcd7735_sendCommand(0x14);
	// display on
	lcd7735_sendCommand(0x29);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){
	
	/*	SPI2 GPIO Configuration
			PB10 	---> SPI2_SCK
			PC3		---> SPI2_MOSI	
	*/
	
	__SPI2_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	
	GPIO_InitTypeDef SPI2_SCK, SPI2_MOSI;
	
	SPI2_SCK.Pin 	= GPIO_PIN_10;
	SPI2_SCK.Mode = GPIO_MODE_AF_PP;
	SPI2_SCK.Pull = GPIO_NOPULL;
	SPI2_SCK.Speed = GPIO_SPEED_HIGH;
	SPI2_SCK.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &SPI2_SCK);
	
	SPI2_MOSI.Pin 	= GPIO_PIN_3;
	SPI2_MOSI.Mode = GPIO_MODE_AF_PP;
	SPI2_MOSI.Pull = GPIO_NOPULL;
	SPI2_MOSI.Speed = GPIO_SPEED_HIGH;
	SPI2_MOSI.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOC, &SPI2_MOSI);
	
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi){
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3	);
}