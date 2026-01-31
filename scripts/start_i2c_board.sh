#!/bin/bash
# Start the i2c PWM board node for servo control
# Run this on the Raspberry Pi first

echo "Starting i2cpwm_board node..."
echo "This controls the PCA9685 servo board via I2C"
echo "----------------------------------------"

# Source ROS environment
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Run the i2c PWM board node
rosrun i2cpwm_board i2cpwm_board
