#ifndef MPU5060_LIBRARY_H
#define MPU5060_LIBRARY_H

#include <array>

class MPU5060 {
public:
    /// Initialize object for MPU5060
    /// \param i2c_bus The I2C bus number.
    /// \param device_address The address of the MPU5060.
    /// \param power_management_reg_address The address of the power management register.
    explicit MPU5060(int i2c_bus = 1, int device_address = 0x68, int power_management_reg_address = 0x6B);

    /// Connects to the I2C device
    /// \return Returns 1 for success and -1 for unsuccessfully connection
    int connect();

    /// Reads the linear acceleration. Unit: m/s^2
    /// Assuming at <address> is the high bytes and <address + 1> is the low bytes.
    /// \param x_address The address of the register for x-acceleration.
    /// \param y_address The address of the register for y-acceleration.
    /// \param z_address The address of the register for z-acceleration.
    /// \return Returns The linear accelerations in the order of [x, y, z].
    std::array<float, 3> read_linear_acceleration(int x_address = 0x3B, int y_address = 0x3D, int z_address = 0x3F) const;

    /// Reads the angular velocity. Unit: rad/sec
    /// Assuming at <address> is the high bytes and <address + 1> is the low bytes.
    /// \param x_address The address of the register for x-angular-velocity.
    /// \param y_address The address of the register for y-angular-velocity.
    /// \param z_address The address of the register for z-angular-velocity.
    /// \return Returns The angular velocities in the order of [x, y, z].
    std::array<float, 3> read_angular_velocity(int x_address = 0x43, int y_address = 0x45, int z_address = 0x47) const;

private:
    int i2c_bus_;
    int device_address_;
    int file_descriptor_;
    int power_management_reg_address_;

    /// Reads the data at a register given the address.
    /// \param address The address of the register. Assuming at <address> is the high bytes and <address + 1> is the low bytes.
    /// \return Returns the data at the register.
    int read_data(int address) const;
};

#endif //MPU5060_LIBRARY_H
