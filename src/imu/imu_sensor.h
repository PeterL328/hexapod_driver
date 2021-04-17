#ifndef MPU5060_IMU_SENSOR_H
#define MPU5060_IMU_SENSOR_H

#include <memory>

#include "mpu5060.h"

namespace IMU {
    class IMU_SENSOR {
    private:
        std::unique_ptr<MPU5060> MPU5060_controller;

    public:
        /// Creates an instance of a MPU5060 object.
        explicit IMU_SENSOR();

        /// Reads the linear acceleration. Unit: m/s^2
        /// \return Returns The linear accelerations in the order of [x, y, z].
        std::array<float, 3> read_linear_acceleration();

        /// Reads the angular velocity. Unit: rad/sec
        /// \return Returns The angular velocities in the order of [x, y, z].
        std::array<float, 3> read_angular_velocity();
    };
}
#endif //MPU5060_IMU_SENSOR_H