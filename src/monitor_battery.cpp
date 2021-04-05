#include <iostream>

#include "ros/ros.h"

#include "ads7830/Battery.h"
#include "battery_status.h"

int main(int argc, char **argv)
{
    const std::string node_name = "monitor_battery";
    const std::string battery_level_topic_name = "battery_level";
    const int publish_rate_in_hz = 1;

    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    ros::Publisher battery_level_pub = n.advertise<ads7830::Battery>(battery_level_topic_name, 1000);
    ros::Rate loop_rate(publish_rate_in_hz);

    // Battery configuration setup
    const float li_ion_cut_off_v = 3.0f;
    const float li_ion_max_v = 4.2f;
    Battery::BatteryStatus battery_monitor{li_ion_cut_off_v, li_ion_max_v};

    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        ads7830::Battery msg;

        // Read battery status
        msg.control_board_battery = battery_monitor.read_battery_percentage(Battery::BatteryType::CONTROL_BOARD);
        msg.servo_battery = battery_monitor.read_battery_percentage(Battery::BatteryType::SERVO);

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        battery_level_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
