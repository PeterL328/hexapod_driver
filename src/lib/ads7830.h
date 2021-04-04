#ifndef ADS7830_LIBRARY_H
#define ADS7830_LIBRARY_H

#include <iostream>
#include <string>

class ADS7830 {
private:
    int i2c_bus_;
    int device_address_;
public:
    explicit ADS7830(int i2c_bus = 1, int device_address = 0x48);
    float read_voltage(int channel);
};

#endif //ADS7830_LIBRARY_H
