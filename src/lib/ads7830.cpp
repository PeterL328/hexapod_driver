#include <iostream>
#include <wiringPiI2C.h>

#include "ads7830.h"

ADS7830::ADS7830(int i2c_bus, int device_address) :
    i2c_bus_(i2c_bus), device_address_(device_address) {}

int ADS7830::connect() {
    file_descriptor_ = wiringPiI2CSetup(device_address_);
    if (file_descriptor_ == -1) {
        return -1;
    }
    return 1;
}

float ADS7830::read_voltage(int channel) {
    return 1.1f;
}