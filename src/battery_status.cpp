#include <ros/console.h>

#include "battery_status.h"

namespace Battery {
    BatteryStatus::BatteryStatus(BatteryType battery, float lower_bound, float upper_bound) :
        lower_bound_(lower_bound), upper_bound_(upper_bound) {
        ADS7830_controller = std::make_unique<ADS7830>();
        if (battery == BatteryType::CONTROL_BOARD) {
            channel_ = 4;
        } else if (battery == BatteryType::SERVO) {
            channel_ = 0;
        } else {
            ROS_ERROR("Invalid battery type. ");
            throw std::invalid_argument("Invalid battery type.");
        }
    }

    float BatteryStatus::read_battery_percentage() {
        return ADS7830_controller->read_voltage(channel_);
    }
}