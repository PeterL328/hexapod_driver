#include <algorithm>

#include <ros/console.h>

#include "servo_controller.h"

namespace Servo {
    ServoController::ServoController(int device_address_1, int device_address_2) {
        // Setup the PCA9685 objects
        PCA9685_controller_1 = std::make_unique<PCA9685>(i2c_bus_line_, device_address_1);
        PCA9685_controller_2 = std::make_unique<PCA9685>(i2c_bus_line_, device_address_2);
        // Attempt connection with the I2C devices
        int status_code = PCA9685_controller_1->connect();
        if (status_code == -1) {
            ROS_ERROR("Failed to connect to PCA9685(1) via I2C.");
            throw std::runtime_error("Failed to connect to PCA9685(1) via I2C.");
        }
        PCA9685_controller_1->set_pwm_freq(servo_control_frequency_);
        ROS_INFO("Successfully connected to PCA9685(1) via I2C.");

        status_code = PCA9685_controller_2->connect();
        if (status_code == -1) {
            ROS_ERROR("Failed to connect to PCA9685(2) via I2C.");
            throw std::runtime_error("Failed to connect to PCA9685(2) via I2C.");
        }
        PCA9685_controller_2->set_pwm_freq(servo_control_frequency_);
        ROS_INFO("Successfully connected to PCA9685(2) via I2C.");
    }

    void ServoController::set_angle(int channel, float angle) {
        // Crop the angle to be between the bounds.
        angle = std::max(lower_bound_angle_, std::min(upper_bound_angle_, angle));
        // From experiments, the millisecond bounds are [650, 2700](ms) corresponding to [0, 180](deg).
        float milliseconds_on = ((angle * 11.388f) + 650) / 1000.0f;
        if (channel < 16) {
            PCA9685_controller_2->set_pwm_ms(channel, milliseconds_on);
        } else {
            channel -= 16;
            PCA9685_controller_1->set_pwm_ms(channel, milliseconds_on);
        }
    }
}