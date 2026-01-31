#!/bin/bash
# Interactive servo control - works over SSH
# Uses read instead of raw terminal input

source /opt/ros/kinetic/setup.bash 2>/dev/null || source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/catkin_ws/devel/setup.bash

echo "========================================"
echo "  Interactive Servo Control"
echo "========================================"
echo ""
echo "Commands:"
echo "  <servo> <value>  - Set servo to value"
echo "  center <servo>   - Set servo to center (306)"
echo "  off <servo>      - Turn servo off (freewheel)"
echo "  all center       - All servos to center"
echo "  all off          - All servos off"
echo "  quit             - Exit"
echo ""
echo "Examples:"
echo "  1 350            - Servo 1 to 350"
echo "  center 1         - Servo 1 to center"
echo "========================================"

send_servo() {
    local servo=$1
    local value=$2
    rostopic pub /servos_absolute i2cpwm_board/ServoArray "servos:
- servo: $servo
  value: $value" --once 2>/dev/null
    echo "Servo $servo -> $value"
}

send_all() {
    local value=$1
    rostopic pub /servos_absolute i2cpwm_board/ServoArray "servos:
- {servo: 1, value: $value}
- {servo: 2, value: $value}
- {servo: 3, value: $value}
- {servo: 4, value: $value}
- {servo: 5, value: $value}
- {servo: 6, value: $value}
- {servo: 7, value: $value}
- {servo: 8, value: $value}
- {servo: 9, value: $value}
- {servo: 10, value: $value}
- {servo: 11, value: $value}
- {servo: 12, value: $value}" --once 2>/dev/null
    echo "All servos -> $value"
}

while true; do
    echo -n "> "
    read cmd arg
    
    case "$cmd" in
        quit|exit|q)
            echo "Exiting..."
            break
            ;;
        center)
            if [ -n "$arg" ]; then
                send_servo $arg 306
            else
                echo "Usage: center <servo>"
            fi
            ;;
        off)
            if [ -n "$arg" ]; then
                send_servo $arg 0
            else
                echo "Usage: off <servo>"
            fi
            ;;
        all)
            case "$arg" in
                center) send_all 306 ;;
                off) send_all 0 ;;
                *) echo "Usage: all center|off" ;;
            esac
            ;;
        [0-9]|[0-9][0-9])
            if [ -n "$arg" ]; then
                send_servo $cmd $arg
            else
                echo "Usage: <servo> <value>"
            fi
            ;;
        *)
            echo "Unknown command. Type 'quit' to exit."
            ;;
    esac
done
