#!/bin/bash
# Start the servo keyboard control for calibration
# Run this after start_i2c_board.sh is running

echo "Starting Servo Keyboard Control..."
echo "----------------------------------------"
echo "Commands:"
echo "  oneServo  - Control single servo (others go idle)"
echo "  allServos - Control all servos together"
echo "  quit      - Exit program"
echo ""
echo "Keyboard controls in oneServo mode:"
echo "  y - Center position (PWM 306)"
echo "  g/j - Decrease/Increase by 1"
echo "  f/k - Decrease/Increase by 10"
echo "  z/x - Min/Max position"
echo "  q - Back to menu"
echo "----------------------------------------"

# Source ROS environment
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Run the servo move keyboard node
rosrun servo_move_keyboard servoMoveKeyboard.py
