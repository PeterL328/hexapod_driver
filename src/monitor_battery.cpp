#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"

#include "battery_status.h"

int main(int argc, char **argv)
{
    const std::string node_name = "monitor_battery";
    const std::string battery_level_topic_name = "battery_level";
    const int publish_rate_in_hz = 1;

    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    ros::Publisher battery_level_pub = n.advertise<sensor_msgs::BatteryState>(battery_level_topic_name, 1000);
    ros::Rate loop_rate(publish_rate_in_hz);

    // Battery configuration setup (Two li-ion cells in series)
    const float li_ion_cut_off_v = 6.0f;
    const float li_ion_max_v = 8.4f;
    Battery::BatteryStatus battery_monitor{li_ion_cut_off_v, li_ion_max_v};

    while (ros::ok())
    {
        // Create message objects
        sensor_msgs::BatteryState control_board_msg;
        sensor_msgs::BatteryState servo_msg;

        // Read battery status
        std::pair<float, float> battery_status_control_board = battery_monitor.read_battery_percentage(Battery::BatteryType::CONTROL_BOARD);
        std::pair<float, float> battery_status_servo = battery_monitor.read_battery_percentage(Battery::BatteryType::SERVO);

        // Populating the control board battery message
        control_board_msg.voltage = battery_status_control_board.first;
        control_board_msg.percentage = battery_status_control_board.second;
        control_board_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        control_board_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        control_board_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
        control_board_msg.location = "control_board";
        control_board_msg.present = true;

        // Populating the servo battery message
        servo_msg.voltage = battery_status_servo.first;
        servo_msg.percentage = battery_status_servo.second;
        servo_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        servo_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        servo_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
        servo_msg.location = "servo";
        servo_msg.present = true;

        // Publish message on the topic
        battery_level_pub.publish(control_board_msg);
        battery_level_pub.publish(servo_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
