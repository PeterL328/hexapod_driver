#include "ros/ros.h"
#include <ros/console.h>

#include "servo_controller.h"

int main(int argc, char **argv)
{
    const std::string node_name = "test_joint_controller";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    // Get values from arguments
    float publish_rate_in_hz;
    int channel;
    n.param("publish_rate", publish_rate_in_hz, 0.3f);
    ROS_INFO("Publishing at %fHz.", publish_rate_in_hz);
    n.param("channel", channel, 1);

    ros::Rate loop_rate(publish_rate_in_hz);

    // Setup servo controller object
    Servo::ServoController servo_controller{};

    int i = 0;
    while (ros::ok())
    {
        int upper_angle = 180;
        int lower_angle = 0;
        if (i % 2 == 0) {
            ROS_INFO("Setting angle to %d degrees on channel: %d.", upper_angle, channel);
            servo_controller.set_angle(channel, upper_angle);
        } else {
            ROS_INFO("Setting angle to %d degrees on channel: %d.", lower_angle, channel);
            servo_controller.set_angle(channel, lower_angle);
        }
        i++;
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
