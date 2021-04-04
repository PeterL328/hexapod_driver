#include "ads7830.h"

#include <iostream>
#include <string>

ADS7830::ADS7830(int i2c_bus, int device_address) :
    i2c_bus_(i2c_bus), device_address_(device_address) {}

float ADS7830::read_voltage(int channel) {
    return 1.1f;
}