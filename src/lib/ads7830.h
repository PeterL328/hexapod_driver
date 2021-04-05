#ifndef ADS7830_LIBRARY_H
#define ADS7830_LIBRARY_H

class ADS7830 {
private:
    int i2c_bus_;
    int device_address_;
    int file_descriptor_;
public:
    ///
    /// \param i2c_bus The I2C bus number.
    /// \param device_address The address of the ADS7830.
    explicit ADS7830(int i2c_bus = 1, int device_address = 0x48);

    /// Connects to the I2C device
    /// \return Returns 1 for success and -1 for unsuccessfully connection
    int connect();

    /// Reads the voltage at a given channel.
    /// \param channel The channel to read the voltage.
    /// \return Returns the voltage at the given channel.
    float read_voltage(int channel);
};

#endif //ADS7830_LIBRARY_H
