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