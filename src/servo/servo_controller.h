#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "pca9685.h"

namespace Servo {
    class ServoController {
    public:
        /// Creates an instance of a ServoController object.
        /// \param device_address_1 The address of the PCA9685 controller for channel [0 - 15] inclusive
        /// \param device_address_2 The address of the PCA9685 controller for channel [16 - 31] inclusive
        explicit ServoController(int device_address_1 = 0x40, int device_address_2 = 0x41);

        /// Sets the angle[deg] for a servo at a channel
        /// The angle will be cropped between the lower_bound and the upper_bound.
        /// \param channel
        /// \param angle
        void set_angle(int channel, float angle);

    private:
        PCA9685 PCA9685_controller_1;
        PCA9685 PCA9685_controller_2;
        const int i2c_bus_line_{1};
        const float lower_bound_angle_{0.0f};
        const float upper_bound_angle_{180.0f};
        const float servo_control_frequency_{50.0f};
    };
}
#endif //SERVO_CONTROLLER_H