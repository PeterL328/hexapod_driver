#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "mpu5060.h"

namespace Imu {
    class ImuSensor {
    public:
        /// Creates an instance of a MPU5060 object.
        explicit ImuSensor();

        /// Reads the linear acceleration. Unit: m/s^2
        /// \return Returns The linear accelerations in the order of [x, y, z].
        std::array<float, 3> read_linear_acceleration() const;

        /// Reads the angular velocity. Unit: rad/sec
        /// \return Returns The angular velocities in the order of [x, y, z].
        std::array<float, 3> read_angular_velocity() const;

    private:
        MPU5060 MPU5060_controller_;
    };
}
#endif //IMU_SENSOR_H