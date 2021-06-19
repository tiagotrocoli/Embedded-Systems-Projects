#include "stm32f4xx.h"                  // Device header
#include <stdio.h>
#include "i2c.h"
#include "TJ_MPU6050.h"

// MPU6050 I2C address
// the LSB bit is zero (master write operation)
#define MPU6050_I2C_ADDRS (0x68 << 1) // 0xD0

UART_HandleTypeDef huart5;
I2C_HandleTypeDef hi2c1;
ScaledData_Def myAccelScaled, myGyroScaled;

void Error_Handler(void);
static void MX_UART5_Init(void);
void printmsg(char *);

char usr_msg[250]={0};

int main(void){
	float x = 0, y = 0, z = 0;
	MPU_ConfigTypeDef myMpuConfig;
	
	i2c_I2C1_GPIO_config();
	i2c_I2C1_config();
	MX_UART5_Init();
	
	uint8_t count = 1;
	if(i2c_I2C1_isSlaveAddressExist(MPU6050_I2C_ADDRS)){
		
		sprintf(usr_msg, "Initializing...\r\n");
		printmsg(usr_msg);
		
		MPU6050_Init(&hi2c1);
		//2. Configure Accel and Gyro parameters
		myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
		myMpuConfig.ClockSource = Internal_8MHz;
		myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
		myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
		myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
		MPU6050_Config(&myMpuConfig);
		
		while(1){
			MPU6050_Get_Accel_Scale(&myAccelScaled);
			x = myAccelScaled.x;
			y = myAccelScaled.y;
			z = myAccelScaled.z;
			sprintf(usr_msg, "Acce: %0.2f, %0.2f, %0.2f \r\n", x, y, z);
			printmsg(usr_msg);
			//HAL_Delay(500);
		}
		
	}else{
		while(count--);
	}
	
	
	while(1);
	
	
}

static void MX_UART5_Init(void){

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart5) != HAL_OK){
	  Error_Handler();
  }
}

void printmsg(char *msg){
	HAL_UART_Transmit(&huart5, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void Error_Handler(void){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while(1);
  /* USER CODE END Error_Handler_Debug */
}

