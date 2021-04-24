#include <array>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

#include "imu_sensor.h"

int main(int argc, char **argv)
{
    const std::string node_name = "imu_readings";
    const std::string imu_raw_topic_name = "imu_raw";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    // Get values from arguments
    float publish_rate_in_hz;
    n.param("publish_rate", publish_rate_in_hz, 1.0f);
    ROS_INFO("Publishing at %fHz.", publish_rate_in_hz);

    ros::Publisher imu_raw_pub = n.advertise<sensor_msgs::Imu>(imu_raw_topic_name, 1000);
    ros::Rate loop_rate(publish_rate_in_hz);

    Imu::ImuSensor imu{};

    while (ros::ok())
    {
        // Create message objects
        sensor_msgs::Imu Imu_raw_msg;

        // Read imu data
        std::array<float, 3> acceleration_readings = imu.read_linear_acceleration();
        std::array<float, 3> angular_vel_readings = imu.read_angular_velocity();

        // Get current time
        ros::Time current_timestamp = ros::Time::now();
        Imu_raw_msg.header.stamp = current_timestamp;

        // Configure message
        Imu_raw_msg.linear_acceleration.x = acceleration_readings[0];
        Imu_raw_msg.linear_acceleration.y = acceleration_readings[1];
        Imu_raw_msg.linear_acceleration.z = acceleration_readings[2];

        Imu_raw_msg.angular_velocity.x = angular_vel_readings[0];
        Imu_raw_msg.angular_velocity.y = angular_vel_readings[1];
        Imu_raw_msg.angular_velocity.z = angular_vel_readings[2];

        // Publish message on the topic
        imu_raw_pub.publish(Imu_raw_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
