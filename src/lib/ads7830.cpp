#include <iostream>
#include <wiringPiI2C.h>

#include "ads7830.h"

ADS7830::ADS7830(int i2c_bus, int device_address) :
    i2c_bus_(i2c_bus), device_address_(device_address) {}

int ADS7830::connect() {
    file_descriptor_ = wiringPiI2CSetup(device_address_);
    // TODO: Can still return a positive fd if device is not connected
    if (file_descriptor_ < 0) {
        return -1;
    }
    return 1;
}

float ADS7830::read_voltage(int channel) {
    /* ADS7830 Command Set
     * MSB | 6 | 5 | 4 | 3 | 2 | 1 | LSB |
     *  SD | C2| C1| C0|PD1|PD0| X |  X  |
     *
     *  SD: 0 for Differential Inputs
     *      1 for Single-Ended Inputs (We want to use this one)
     *  C2 - C0: Channel Selection. (0 | 0 | 0 for channel 0 and 0 | 1 | 0 for channel 4.)
     *  For more detail on how channel selection works, see ADS7830 documentation.
     *
     *  PD1 - 0: Power-Down Selection
     *  X: Unused
     */
    int command_set = 0x84 | ((((channel << 2) | (channel >> 1)) & 0x07) << 4);
    wiringPiI2CWrite(file_descriptor_, command_set);
    int data = wiringPiI2CRead(file_descriptor_);
    return (static_cast<float>(data) / 255.0f) * 15.0f;
}
