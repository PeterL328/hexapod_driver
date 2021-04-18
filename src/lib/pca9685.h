#ifndef PCA9685_LIBRARY_H
#define PCA9685_LIBRARY_H

#include <cstdint>

class PCA9685 {
public:
    /// Initialize object for PCA9685.
    /// \param i2c_bus The I2C bus number.
    /// \param device_address The address of the PCA9685.
    explicit PCA9685(int i2c_bus, int device_address);

    /// Initialize object for PCA9685 on the first i2c bus.
    /// \param device_address The address of the PCA9685.
    explicit PCA9685(int device_address = 0x40);

    /// Connects to the I2C device
    /// \return Returns 1 for success and -1 for unsuccessfully connection
    int connect();

    /// Sets the frequency of the pwm signals.
    /// \param frequency_hz The frequency with unit of Hz.
    void set_pwm_freq(float frequency_hz);

    /// Send pwm signal to a channel.
    ///  |         |----------|           |
    ///  |0________|          |-----------|4095
    ///  ---------->ON
    ///  --------------------->OFF
    /// PWM duty cycle = (OFF - ON) / 4096, delay is ON.
    /// \param channel The channel to send the pwm signal.
    /// \param on The number of units from 0 until we switch ON.
    /// \param off The number of units from 0 until we switch OFF.
    void set_pwm(int channel, uint16_t on, uint16_t off);

    /// Send pwm signal to a channel in terms of milliseconds.
    /// This does not have any delay. If delays are needed, use set_pwm().
    /// \param channel The channel to send the pwm signal
    /// \param ms The time in milliseconds to turn ON.
    void set_pwm_ms(int channel, float ms);

    /// Send pwm signal to all channel.
    ///  |         |----------|           |
    ///  |0________|          |-----------|4095
    ///  ---------->ON
    ///  --------------------->OFF
    /// PWM duty cycle = (OFF - ON) / 4096, delay is ON.
    /// \param on The number of units from 0 until we switch ON.
    /// \param off The number of units from 0 until we switch OFF.
    void set_all_pwm(uint16_t on, uint16_t off);

    /// Gets the number of total channels
    /// \return The number of total channels.
    inline int num_channel() {
        return num_channel_;
    }

private:
    int i2c_bus_;
    int device_address_;
    int file_descriptor_;
    float frequency_hz_;
    const int num_channel_{16};
};

#endif //PCA9685_LIBRARY_H
