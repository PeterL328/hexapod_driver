#include <ros/ros.h>
#include <ros/console.h>

#include "joint_controller.h"
#include "joint_constants.h"

JointController::JointController() {
    // Setup the servo controller object
    servo_controller = std::make_unique<Servo::ServoController>();
}

void JointController::legs_state_update(const hexapod_msgs::LegsJoints::ConstPtr &legs_joints) {
    // Preprocess the joint angle
    auto preprocess = [](float angle, bool rotation_dir){
        if (!rotation_dir) {
            return 180.f - angle;
        }
        return angle;
    };

    // Left front leg
    servo_controller->set_angle(LF_COXA_CH, preprocess(legs_joints->left_front_leg.coxa, LF_COXA_DIR));
    servo_controller->set_angle(LF_FEMUR_CH, preprocess(legs_joints->left_front_leg.femur, LF_FEMUR_DIR));
    servo_controller->set_angle(LF_TIBIA_CH, preprocess(legs_joints->left_front_leg.tibia, LF_TIBIA_DIR));

    // Left middle leg
    servo_controller->set_angle(LM_COXA_CH, preprocess(legs_joints->left_mid_leg.coxa, LM_COXA_DIR));
    servo_controller->set_angle(LM_FEMUR_CH, preprocess(legs_joints->left_mid_leg.femur, LM_FEMUR_DIR));
    servo_controller->set_angle(LM_TIBIA_CH, preprocess(legs_joints->left_mid_leg.tibia, LM_TIBIA_DIR));

    // Left back leg
    servo_controller->set_angle(LB_COXA_CH, preprocess(legs_joints->left_back_leg.coxa, LB_COXA_DIR));
    servo_controller->set_angle(LB_FEMUR_CH, preprocess(legs_joints->left_back_leg.femur, LB_FEMUR_DIR));
    servo_controller->set_angle(LB_TIBIA_CH, preprocess(legs_joints->left_back_leg.tibia, LB_TIBIA_DIR));

    // Right front leg
    servo_controller->set_angle(RF_COXA_CH, preprocess(legs_joints->right_front_leg.coxa, RF_COXA_DIR));
    servo_controller->set_angle(RF_FEMUR_CH, preprocess(legs_joints->right_front_leg.femur, RF_FEMUR_DIR));
    servo_controller->set_angle(RF_TIBIA_CH, preprocess(legs_joints->right_front_leg.tibia, RF_TIBIA_DIR));

    // Right middle leg
    servo_controller->set_angle(RM_COXA_CH, preprocess(legs_joints->right_mid_leg.coxa, RM_COXA_DIR));
    servo_controller->set_angle(RM_FEMUR_CH, preprocess(legs_joints->right_mid_leg.femur, RM_FEMUR_DIR));
    servo_controller->set_angle(RM_TIBIA_CH, preprocess(legs_joints->right_mid_leg.tibia, RM_TIBIA_DIR));

    // Right back leg
    servo_controller->set_angle(RB_COXA_CH, preprocess(legs_joints->right_back_leg.coxa, RB_COXA_DIR));
    servo_controller->set_angle(RB_FEMUR_CH, preprocess(legs_joints->right_back_leg.femur, RB_FEMUR_DIR));
    servo_controller->set_angle(RB_TIBIA_CH, preprocess(legs_joints->right_back_leg.tibia, RB_TIBIA_DIR));
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
