#include <ros/console.h>

#include "imu_sensor.h"

namespace IMU {
    IMU_SENSOR::IMU_SENSOR() {
        // Setup the MPU5060 object
        MPU5060_controller = std::make_unique<MPU5060>();
        // Attempt connection with the I2C device (MPU5060)
        int status_code = MPU5060_controller->connect();
        if (status_code == -1) {
            ROS_ERROR("Failed to connect to MPU5060 via I2C.");
            throw std::runtime_error("Failed to connect to MPU5060 via I2C.");
        }
        ROS_INFO("Successfully connected to MPU5060 via I2C.");
    }

    std::array<float, 3> IMU_SENSOR::read_linear_acceleration() {
        return MPU5060_controller->read_linear_acceleration();
    }

    std::array<float, 3> IMU_SENSOR::read_angular_velocity() {
        return MPU5060_controller->read_angular_velocity();
    }
}