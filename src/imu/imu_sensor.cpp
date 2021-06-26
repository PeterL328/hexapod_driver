#include <ros/console.h>

#include "imu_sensor.h"

namespace Imu {
    ImuSensor::ImuSensor() : MPU5060_controller_() {
        // Attempt connection with the I2C device (MPU5060)
        int status_code = MPU5060_controller_.connect();
        if (status_code == -1) {
            ROS_ERROR("Failed to connect to MPU5060 via I2C.");
            throw std::runtime_error("Failed to connect to MPU5060 via I2C.");
        }
        ROS_INFO("Successfully connected to MPU5060 via I2C.");
    }

    std::array<float, 3> ImuSensor::read_linear_acceleration() const {
        return MPU5060_controller_.read_linear_acceleration();
    }

    std::array<float, 3> ImuSensor::read_angular_velocity() const {
        return MPU5060_controller_.read_angular_velocity();
    }
}