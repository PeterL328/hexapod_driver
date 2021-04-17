#include <wiringPiI2C.h>

#include "mpu5060.h"

MPU5060::MPU5060(int i2c_bus, int device_address, int power_management_reg_address) :
        i2c_bus_(i2c_bus), device_address_(device_address), power_management_reg_address_(power_management_reg_address) {}

int MPU5060::connect() {
    file_descriptor_ = wiringPiI2CSetup(device_address_);
    // TODO: Can still return a positive fd if device is not connected
    if (file_descriptor_ < 0) {
        return -1;
    }
    // Device starts in sleep mode, so wake it up.
    wiringPiI2CWriteReg16(file_descriptor_, power_management_reg_address_, 0);
    return 1;
}

std::array<float, 3> MPU5060::read_linear_acceleration(int x_address, int y_address, int z_address) {
    // At default sensitivity of 2g we need to scale by 16384.
    // and since we want to units to be m/s^2, we need to divide by g.
    const float scaling_factor = 16384 / 9.807;
    // The data array layout is [x, y, z].
    std::array<float, 3> data;
    data[0] = read_data(x_address) / scaling_factor;
    data[1] = read_data(y_address) / scaling_factor;
    data[2] = read_data(z_address) / scaling_factor;
    return data;
}

std::array<float, 3> MPU5060::read_angular_velocity(int x_address, int y_address, int z_address) {
    // At default sensitivity of 250deg/s we need to scale by 131.
    const float scaling_factor = 131.0f;
    // The data array layout is [x, y, z].
    std::array<float, 3> data;
    data[0] = read_data(x_address) / scaling_factor;
    data[1] = read_data(y_address) / scaling_factor;
    data[2] = read_data(z_address) / scaling_factor;
    return data;
}

int MPU5060::read_data(int address) {
    int high = wiringPiI2CReadReg8(file_descriptor_, address);
    int low = wiringPiI2CReadReg8(file_descriptor_, address + 1);
    int data = (high << 8) + low;
    if (data >= 32768) {
        return -((65535 - data) + 1);
    }
    return data;
}
