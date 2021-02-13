/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: 15 de mai de 2020
 *      Author: tiago
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

// This could be wrong, I think it is uint32_t, GPIO_Init
typedef struct{
	uint8_t GPIO_PinNumber;			/* Possible values from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;			/* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* Possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;	/* Possible values from @GPIO_PIN_PuPdControl */
	uint8_t GPIO_PinOPType;			/* Possible values from @GPIO_PIN_OUTPUT */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct{
	// pointer to hold the base address of GPIO peripheral
	GPIO_RegDef_t *pGPIOx; //This holds the base address to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // this holds GPIO configuration settings
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBER
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15



/*
 * @GPIO_PIN_MODES
 * GPIO pin possible input modes
 */

// IT = interrupt mode, FT = falling edge, RT rising edge, RFT = rising edge and falling edge
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/* @GPIO_PIN_OUTPUT
 * GPIO pin possible output modes
 */
#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD		1

/* @GPIO_PIN_SPEED
 * GPIO pin possible output speed modes
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*@GPIO_PIN_PuPdControl
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*
 ****************************************************************************
 * 							APIs Supported by this driver					*
 ****************************************************************************
 */


/*
 * 	Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);  // don't understand the parameter

/*
 * Data read and write
 */
uint8_t 	GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t 	GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void 		GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void 		GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void 		GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
