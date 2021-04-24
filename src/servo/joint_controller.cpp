#include <ros/ros.h>
#include <ros/console.h>

#include "joint_controller.h"
#include "joint_servo_mapping.h"

JointController::JointController() {
    // Setup the servo controller object
    servo_controller = std::make_unique<Servo::ServoController>();
}

void JointController::legs_state_update(const hexapod_msgs::LegsJoints::ConstPtr &legs_joints) {
    // Left front leg
    servo_controller->set_angle(LF_COXA_CH, legs_joints->left_front_leg.coxa);
    servo_controller->set_angle(LF_FEMUR_CH, legs_joints->left_front_leg.femur);
    servo_controller->set_angle(LF_TIBIA_CH, legs_joints->left_front_leg.tibia);

    // Left middle leg
    servo_controller->set_angle(LM_COXA_CH, legs_joints->left_mid_leg.coxa);
    servo_controller->set_angle(LM_FEMUR_CH, legs_joints->left_mid_leg.femur);
    servo_controller->set_angle(LM_TIBIA_CH, legs_joints->left_mid_leg.tibia);

    // Left back leg
    servo_controller->set_angle(LB_COXA_CH, legs_joints->left_back_leg.coxa);
    servo_controller->set_angle(LB_FEMUR_CH, legs_joints->left_back_leg.femur);
    servo_controller->set_angle(LB_TIBIA_CH, legs_joints->left_back_leg.tibia);

    // Right front leg
    servo_controller->set_angle(RF_COXA_CH, legs_joints->right_front_leg.coxa);
    servo_controller->set_angle(RF_FEMUR_CH, legs_joints->right_front_leg.femur);
    servo_controller->set_angle(RF_TIBIA_CH, legs_joints->right_front_leg.tibia);

    // Right middle leg
    servo_controller->set_angle(RM_COXA_CH, legs_joints->right_mid_leg.coxa);
    servo_controller->set_angle(RM_FEMUR_CH, legs_joints->right_mid_leg.femur);
    servo_controller->set_angle(RM_TIBIA_CH, legs_joints->right_mid_leg.tibia);

    // Right back leg
    servo_controller->set_angle(RB_COXA_CH, legs_joints->right_back_leg.coxa);
    servo_controller->set_angle(RB_FEMUR_CH, legs_joints->right_back_leg.femur);
    servo_controller->set_angle(RB_TIBIA_CH, legs_joints->right_back_leg.tibia);
}

int main(int argc, char **argv)
{
    const std::string node_name = "joint_controller";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    // Setup servo controller object
    JointController joint_controller{};

    ros::Subscriber sub = n.subscribe("joints_state", 1, &JointController::legs_state_update, &joint_controller);

    ros::spin();
    return 0;
}
