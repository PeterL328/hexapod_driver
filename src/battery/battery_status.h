#ifndef BATTERY_STATUS_H
#define BATTERY_STATUS_H

#include <memory>

#include "ads7830.h"

namespace Battery {
    enum class BatteryType {
        CONTROL_BOARD,
        SERVO
    };
    class BatteryStatus {
    public:
        /// Creates an instance of a BatteryStatus object.
        /// \param lower_bound The cut-off voltage of the battery cell.
        /// \param upper_bound The maximum voltage of the battery cell.
        explicit BatteryStatus(float lower_bound, float upper_bound);

        /// Read the battery level in percentage
        /// \param battery Indicates which battery is used
        /// \return Returns the battery percentage in a pair of value.
        ///         The first value is the battery status in voltage and the second is in percentage (decimal).
        ///         If the voltage is faulty, then a -1.0f voltage will be be returned
        std::pair<float, float> read_battery_percentage(BatteryType battery) const;

    private:
        std::unique_ptr<ADS7830> ADS7830_controller;
        float lower_bound_;
        float upper_bound_;

    };
}
#endif //BATTERY_STATUS_H