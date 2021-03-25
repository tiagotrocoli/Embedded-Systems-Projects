#ifndef __DEVICE_H
#define __DEVICE_H

#include "address.h"
#include <stdbool.h>

typedef bool State_T;

/* Pointer to an incomplete type, this hides implementation details. */
typedef struct Device* DevicePtr;

DevicePtr createDevice(const char* name, const Address *address);

void turnOnDevice(DevicePtr device);
void turnOffDevice(DevicePtr device);
void toggleDevice(DevicePtr device);

State_T readDevice(DevicePtr device);
void destroyDevice(DevicePtr device);
void displayDeviceInfo(DevicePtr device);

#endif
