# Hexapod driver

## About The Project
This ROS package is used by the [Hexapod project](https://github.com/PeterL328/hexapod) and contains ROS nodes for using different on-board sensors.
At the moment, the package is meant for the [Freenove Big Hexapod kit](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi) and is meant for their custom Raspberry-pi shield.

Included sensors:
- ADS7830
- MPU5060
- PCA9685

## Getting Started
To use this package, ROS needs to be installed on your Raspberry-pi.
For more detail about ROS, please refer to the [ROS documentations](http://wiki.ros.org/).
This project was built with ROS Noetic release.

For communicating over I2C and GPIO, install the Wiring Pi library:
```bash
sudo apt-get install wiringpi
```

## Usage
Using this ROS package is the same with any other ROS package, first create a catkin workspace and clone this repo in the src directory.
Please refer to the [ROS documentations](http://wiki.ros.org/) for more details.
