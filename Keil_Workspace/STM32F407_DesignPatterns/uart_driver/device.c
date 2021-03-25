#include "device.h"
#include "uart.h"
#include <stdlib.h>

typedef uint32_t deviceID_T;

uint32_t noOfdevices = 0;

struct Device{
	const	char *name;
	Address address;
	deviceID_T uuid;
};

DevicePtr createDevice(const char* name, const Address *address){
	
	DevicePtr device = (DevicePtr) calloc(sizeof(DevicePtr), sizeof(DevicePtr));
	
	if(device){
		noOfdevices++;
		device->name = name;
		device->address.Port = address->Port;
		device->address.Pin = address->Pin;
		device->uuid = noOfdevices;
		switch((int) device->address.Port){
			case (int)GPIOA : RCC_AHB1ENR |= GPIOA_EN; break;
			case (int)GPIOB : RCC_AHB1ENR |= GPIOB_EN; break;
			case (int)GPIOC : RCC_AHB1ENR |= GPIOC_EN; break;
			case (int)GPIOD : RCC_AHB1ENR |= GPIOD_EN; break;
			case (int)GPIOE : RCC_AHB1ENR |= GPIOE_EN; break;
			case (int)GPIOH : RCC_AHB1ENR |= GPIOH_EN; break;
		}
	}else{
		printf("Low memory, cannot create device\r\n");
	}
	
	return device;
}

void turnOnDevice(DevicePtr device){
	
	/* 1. COnfigure device as an output device */
	uint16_t _pin = (uint16_t) device->address.Pin;
	device->address.Port->MODER &= ~(0x3U << (2*_pin));
	device->address.Port->MODER |= (0x1 << (2*_pin) );
	
	/* 2. Turn on device */
	device->address.Port->ODR |= (1U << _pin);
	printf("%s is on \n\r", device->name);
	
	
}

void turnOffDevice(DevicePtr device){
	
	uint16_t _pin = (uint16_t) device->address.Pin;
	
	/* Turn off device */
	device->address.Port->ODR &= ~(1U << _pin);
	printf("%s is off \n\r", device->name);
	
	
}

void toggleDevice(DevicePtr device){
	
	uint16_t _pin = (uint16_t) device->address.Pin;
	
	/* Turn off device */
	device->address.Port->ODR ^= (1U << _pin);
	printf("%s has toggled \n\r", device->name);
	
	
}

State_T readDevice(DevicePtr device){
	
	State_T bitStatus;
	
	/* 1. COnfigure device as an input device */
	uint16_t _pin = (uint16_t) device->address.Pin;
	device->address.Port->MODER &= ~(0x3U << (2*_pin));
	device->address.Port->MODER |= (0x0U << (2*_pin) );
	
	// read device
	if(device->address.Port->IDR & (1U << _pin) ){
		bitStatus = 1;
	}else{
		bitStatus = 0;	
	}
	
	return bitStatus;
}

void destroyDevice(DevicePtr device){
	
	printf("*** %s destroyed ***\n\r", device->name);
	free(device);
	
}

void displayDeviceInfo(DevicePtr device){
	
	const char* type;
	uint16_t _pin = (uint16_t) device->address.Pin;

	if( (device->address.Port->MODER & (1U << 2*_pin)) == 1 ){
		type = "Output device";
	}else{
		type = "input device";
	}
	
	printf("**********************************************\n\r");
	printf("Device name: %s\n\r", device->name);
	printf("Device type: %s\n\r", type);
	printf("The device uuid is %d\n\r", device->uuid);
	printf("**********************************************\n\r");
	
	
}
