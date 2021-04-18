#include "ros/ros.h"
#include <ros/console.h>

#include "servo_controller.h"

int main(int argc, char **argv)
{
    const std::string node_name = "joint_controller";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    // Get values from arguments
    float publish_rate_in_hz;
    n.param("publish_rate", publish_rate_in_hz, 5.0f);
    ROS_INFO("Publishing at %fHz.", publish_rate_in_hz);

    ros::Rate loop_rate(publish_rate_in_hz);

    // Setup servo controller object
    Servo::ServoController servo_controller{};

    int i = 0;
    while (ros::ok())
    {
        if (i % 2 == 0) {
            servo_controller.set_angle(17, 0);
        } else {
            servo_controller.set_angle(17, 180);
        }
        i++;
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
