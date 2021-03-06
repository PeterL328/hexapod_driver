#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>

#include "joint_controller.h"
#include "joint_constants.h"

#define _USE_MATH_DEFINES

JointController::JointController() : servo_controller_() {
}

void JointController::legs_state_update_callback(const hexapod_msgs::LegsJoints::ConstPtr &legs_joints) {
    // Left front leg
    servo_controller_.set_angle(LF_COXA_CH, preprocess(legs_joints->left_front_leg.coxa, LF_COXA_DIR, LF_COXA_OFFSET));
    servo_controller_.set_angle(LF_FEMUR_CH, preprocess(legs_joints->left_front_leg.femur, LF_FEMUR_DIR, LF_FEMUR_OFFSET));
    servo_controller_.set_angle(LF_TIBIA_CH, preprocess(legs_joints->left_front_leg.tibia, LF_TIBIA_DIR, LF_TIBIA_OFFSET));

    // Left middle leg
    servo_controller_.set_angle(LM_COXA_CH, preprocess(legs_joints->left_mid_leg.coxa, LM_COXA_DIR, LM_COXA_OFFSET));
    servo_controller_.set_angle(LM_FEMUR_CH, preprocess(legs_joints->left_mid_leg.femur, LM_FEMUR_DIR, LM_FEMUR_OFFSET));
    servo_controller_.set_angle(LM_TIBIA_CH, preprocess(legs_joints->left_mid_leg.tibia, LM_TIBIA_DIR, LM_TIBIA_OFFSET));

    // Left back leg
    servo_controller_.set_angle(LB_COXA_CH, preprocess(legs_joints->left_back_leg.coxa, LB_COXA_DIR, LB_COXA_OFFSET));
    servo_controller_.set_angle(LB_FEMUR_CH, preprocess(legs_joints->left_back_leg.femur, LB_FEMUR_DIR, LB_FEMUR_OFFSET));
    servo_controller_.set_angle(LB_TIBIA_CH, preprocess(legs_joints->left_back_leg.tibia, LB_TIBIA_DIR, LB_TIBIA_OFFSET));

    // Right front leg
    servo_controller_.set_angle(RF_COXA_CH, preprocess(legs_joints->right_front_leg.coxa, RF_COXA_DIR, RF_COXA_OFFSET));
    servo_controller_.set_angle(RF_FEMUR_CH, preprocess(legs_joints->right_front_leg.femur, RF_FEMUR_DIR, RF_FEMUR_OFFSET));
    servo_controller_.set_angle(RF_TIBIA_CH, preprocess(legs_joints->right_front_leg.tibia, RF_TIBIA_DIR, RF_TIBIA_OFFSET));

    // Right middle leg
    servo_controller_.set_angle(RM_COXA_CH, preprocess(legs_joints->right_mid_leg.coxa, RM_COXA_DIR, RM_COXA_OFFSET));
    servo_controller_.set_angle(RM_FEMUR_CH, preprocess(legs_joints->right_mid_leg.femur, RM_FEMUR_DIR, RM_FEMUR_OFFSET));
    servo_controller_.set_angle(RM_TIBIA_CH, preprocess(legs_joints->right_mid_leg.tibia, RM_TIBIA_DIR, RM_TIBIA_OFFSET));

    // Right back leg
    servo_controller_.set_angle(RB_COXA_CH, preprocess(legs_joints->right_back_leg.coxa, RB_COXA_DIR, RB_COXA_OFFSET));
    servo_controller_.set_angle(RB_FEMUR_CH, preprocess(legs_joints->right_back_leg.femur, RB_FEMUR_DIR, RB_FEMUR_OFFSET));
    servo_controller_.set_angle(RB_TIBIA_CH, preprocess(legs_joints->right_back_leg.tibia, RB_TIBIA_DIR, RB_TIBIA_OFFSET));
}

void JointController::head_state_update_callback(const hexapod_msgs::HeadJoints::ConstPtr &head_joints) {
    servo_controller_.set_angle(HEAD_UPDOWN_CH, preprocess(head_joints->up_down, HEAD_UPDOWN_DIR, HEAD_UPDOWN_OFFSET));
    servo_controller_.set_angle(HEAD_LEFTRIGHT_CH, preprocess(head_joints->left_right, HEAD_LEFTRIGHT_DIR, HEAD_LEFTRIGHT_OFFSET));
}

float JointController::preprocess(float angle_rad, bool rotation_dir, float offset_deg){
    // Preprocess the joint angle
    // The angles that we are receiving are in radians
    // We need to convert from [-1.5708 rad, +1.5708 rad] -> [0 deg, 180 deg]
    // 1) Convert from rad to degrees
    float angle_deg = angle_rad * (180.f / M_PI);
    // 2) Shift so [-90, 90] -> [0, 180]
    angle_deg += 90.f;
    // 3) Rotate if needed.
    if (!rotation_dir) {
        angle_deg = 180.f - angle_deg;
    }
    // 4) Apply offsets.
    angle_deg += offset_deg;
    return angle_deg;
}

int main(int argc, char **argv)
{
    const std::string node_name = "joint_controller";
    const std::string joints_command_topic_name = "joints_command";
    const std::string head_joints_command_topic_name = "head_joints_command";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    // Setup servo controller object
    JointController joint_controller{};

    ros::Subscriber joints_sub = n.subscribe(joints_command_topic_name, 1, &JointController::legs_state_update_callback, &joint_controller);
    ros::Subscriber head_joints_sub = n.subscribe(head_joints_command_topic_name, 1, &JointController::head_state_update_callback, &joint_controller);

    ros::spin();
    return 0;
}
