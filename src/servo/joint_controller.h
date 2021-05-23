#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <memory>

#include <hexapod_msgs/LegsJoints.h>

#include "servo_controller.h"

class JointController {
public:
    /// Creates an instance of a JointController object.
    explicit JointController();

    /// Updates the all the leg joints.
    /// \param legs_joints A reference to the LegJoints message.
    void legs_state_update_callback(const hexapod_msgs::LegsJoints::ConstPtr &legs_joints);

    /// Update the head joints
    /// \param head_joints: reference to the HeadJoints message.
    void head_state_update_callback(const hexapod_msgs::HeadJoints::ConstPrt &head_joins);

private:
    std::unique_ptr<Servo::ServoController> servo_controller;
    float preprocess(float angle_deg, bool rotation_dir, float offset_deg);
};
#endif //JOINT_CONTROLLER_H