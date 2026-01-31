# Keyboard Commands

This document describes all keyboard commands available for controlling the Spot Micro robot.

## Robot Motion Control

Use the `spot_micro_keyboard_command` node for controlling robot motion (walking, standing, body angles).

### Launch
```bash
roslaunch spot_micro_keyboard_command keyboard_command.launch
```

### Mode Commands

Type these commands at the prompt and press Enter:

| Command | Description |
|---------|-------------|
| `walk` | Start walk mode and enable keyboard motion control |
| `stand` | Stand the robot up |
| `idle` | Lay the robot down |
| `angle_cmd` | Enter body angle control mode |
| `quit` | Stop and exit the program |

### Walk Mode Controls

After entering `walk` mode, use these keys for motion control:

```
   q   w   e
   a   s   d
       f
```

| Key | Action |
|-----|--------|
| `w` | Increase forward speed (+0.02 m/s) |
| `s` | Decrease forward speed / move backward (-0.02 m/s) |
| `a` | Strafe left (-0.02 m/s) |
| `d` | Strafe right (+0.02 m/s) |
| `q` | Turn left (decrease yaw rate by 3 deg/s) |
| `e` | Turn right (increase yaw rate by 3 deg/s) |
| `f` | Zero all speed commands (stop motion) |
| `u` | Exit walk mode and return to stand |

### Angle Command Mode Controls

After entering `angle_cmd` mode, use these keys to adjust body orientation:

```
   q   w   e
   a   s   d
```

| Key | Action |
|-----|--------|
| `w` | Pitch forward (decrease pitch by 2.5 deg) |
| `s` | Pitch backward (increase pitch by 2.5 deg) |
| `a` | Roll left (decrease roll by 2.5 deg) |
| `d` | Roll right (increase roll by 2.5 deg) |
| `q` | Yaw left (increase yaw by 2.5 deg) |
| `e` | Yaw right (decrease yaw by 2.5 deg) |
| `u` | Exit angle command mode |

---

## Servo Calibration Control

Use the `servo_move_keyboard` node for individual servo control and calibration.

### Launch
```bash
roslaunch servo_move_keyboard keyboard_move.launch
```

### Mode Commands

Type these commands at the prompt and press Enter:

| Command | Description |
|---------|-------------|
| `oneServo` | Control one servo at a time (others set to off/freewheel) |
| `allServos` | Control all 12 servos simultaneously |
| `quit` | Stop and exit the program |

### Servo Control Keys

```
   q                y    
            f   g       j   k
    z   x       b   n   m
```

| Key | Action |
|-----|--------|
| `z` | Set servo to minimum value |
| `y` | Set servo to center value |
| `x` | Set servo to maximum value |
| `f` | Decrease servo value by 10 |
| `g` | Decrease servo value by 1 |
| `j` | Increase servo value by 1 |
| `k` | Increase servo value by 10 |
| `b` | Save current position as new minimum |
| `n` | Save current position as new center |
| `m` | Save current position as new maximum |
| `q` | Exit current mode and return to option select |

### Servo Value Reference

- Servo PWM values range from 0 to 4095 (12-bit resolution)
- Default minimum: ~158 (~0.77ms pulse)
- Default center: ~306 (~1.5ms pulse)
- Default maximum: ~450 (~2.2ms pulse)

---

## General Notes

- Press `Ctrl+C` at any time to force quit either program
- The robot should be in a safe position before starting keyboard control
- For walk mode, speed commands accumulate with each keypress
- Use `f` in walk mode to quickly stop all motion
