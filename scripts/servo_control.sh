#!/bin/bash
# Simple servo control script for SSH
# Usage: ./servo_control.sh <servo_number> <pwm_value>
# Example: ./servo_control.sh 1 306

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

if [ $# -lt 2 ]; then
    echo "Servo Control Script"
    echo "===================="
    echo "Usage: ./servo_control.sh <servo> <value>"
    echo ""
    echo "Examples:"
    echo "  ./servo_control.sh 1 306    # Servo 1 to center"
    echo "  ./servo_control.sh 1 400    # Servo 1 to position 400"
    echo "  ./servo_control.sh 1 0      # Servo 1 off (freewheel)"
    echo ""
    echo "Servo mapping:"
    echo "  1: RF_3 (Right Front Knee)"
    echo "  2: RF_2 (Right Front Shoulder)"
    echo "  3: RF_1 (Right Front Hip)"
    echo "  4: RB_3 (Right Back Knee)"
    echo "  5: RB_2 (Right Back Shoulder)"
    echo "  6: RB_1 (Right Back Hip)"
    echo "  7: LB_3 (Left Back Knee)"
    echo "  8: LB_2 (Left Back Shoulder)"
    echo "  9: LB_1 (Left Back Hip)"
    echo " 10: LF_3 (Left Front Knee)"
    echo " 11: LF_2 (Left Front Shoulder)"
    echo " 12: LF_1 (Left Front Hip)"
    echo ""
    echo "PWM values:"
    echo "  306 = center"
    echo "  83  = min"
    echo "  520 = max"
    echo "  0   = off/freewheel"
    exit 1
fi

SERVO=$1
VALUE=$2

echo "Sending servo $SERVO to position $VALUE..."

rostopic pub /servos_absolute i2cpwm_board/ServoArray "servos:
- servo: $SERVO
  value: $VALUE" --once

echo "Done!"
