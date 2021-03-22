#ifndef __ADDRESS_H
#define __ADDRESS_H

#include "REGS.h"

typedef struct{
	GPIO_TypeDef *Port;
	Pin_Type Pin;	
}Address;


#endif