#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <hexapod_msgs/HeadJoints.h>
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
    void head_state_update_callback(const hexapod_msgs::HeadJoints::ConstPtr &head_joints);

private:
    Servo::ServoController servo_controller_;
    float preprocess(float angle_deg, bool rotation_dir, float offset_deg);
};
#endif //JOINT_CONTROLLER_H