#include <ros/ros.h>
#include <ros/console.h>
#include <hexapod_msgs/LegsJoints.h>

#include "joint_servo_mapping.h"
#include "servo_controller.h"

void legsStateUpdate(const hexapod_msgs::LegsJoints::ConstPtr &legs_joints){
    return;
}

int main(int argc, char **argv)
{
    const std::string node_name = "joint_controller";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    // Setup servo controller object
    Servo::ServoController servo_controller{};

    ros::Subscriber sub = n.subscribe("joints_state", 1, legsStateUpdate);

    ros::spin();

    return 0;
}
