/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 7 de jul de 2020
 *      Author: tiago
 */

/* Notes about I2C
 * - Pins have open-drain + pull-up registor
 * - Communication is always initiate by the master in SDA bus.
 * - SDA is the data bus, SCL is the clock bus.
 * - SDA bus only transmits 8 bits only. (no more, no less)
 * - Every 1 byte is followed by 1 ACK
 * - All communication start with S (START) condition and terminate with P (STOP) condition by the master.
 * - START: SDA goes from HIGH to LOW when SCL is HIGH.
 * - STOP: SDA goes from LOW to HIGH when SCL is HIGH.
 * - array of communication: S + (7 bit addr of slave + R/W bit) + (ACK/NACK from slave) + (7 bits of data + ACK/NACK) ... + (P)
 * - The device that generates the Start (S) is the master.
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint8_t		I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t	*pI2Cx;		/*<- This hold base address of I2C peripheral */
	I2C_Config_t	I2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEEDFM4K	400000
#define I2C_SCL_FM2k		200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C related status flags definitions ADDR, BTF, STOPF, BERR, ARLO, AF, OVR, TIME_OUT
 */
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)

// SR = Repeated Start
#define I2C_DISABLE_SR		RESET
#define I2C_ENABLE_SR		SET

/*
 * 	Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pSPIx, uint8_t EnorDi);
/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2Chandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Other peripheral control API
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

#endif
