#include <ros/console.h>

#include "battery_status.h"

namespace Battery {
    BatteryStatus::BatteryStatus(float lower_bound, float upper_bound) :
        lower_bound_(lower_bound), upper_bound_(upper_bound) {
        // Setup the ADS7830 object
        ADS7830_controller = std::make_unique<ADS7830>();
        // Attempt connection with the I2C device (ADS7830)
        int status_code = ADS7830_controller->connect();
        if (status_code == -1) {
            ROS_ERROR("Failed to connect to ADS7830 via I2C.");
            throw std::runtime_error("Failed to connect to ADS7830 via I2C.");
        }
        ROS_INFO("Successfully connected to ADS7830 via I2C.");
    }

    std::pair<float, float> BatteryStatus::read_battery_percentage(BatteryType battery) const {
        int channel = 0;
        if (battery == BatteryType::CONTROL_BOARD) {
            channel = 4;
        } else if (battery == BatteryType::SERVO) {
            channel = 0;
        } else {
            ROS_ERROR("Invalid battery type. ");
            throw std::invalid_argument("Invalid battery type.");
        }
        float voltage = ADS7830_controller->read_voltage(channel);
        // Assuming the voltage is between the lower bound and upper bound.
        float percentage = 0.0f;
        if (voltage <= 0.0f) {
            return {-1.0f, percentage};
        }
        if (voltage >= lower_bound_ && voltage <= upper_bound_) {
            percentage = (voltage - lower_bound_) / (upper_bound_ - lower_bound_);
        }
        return {voltage, percentage};
    }
}