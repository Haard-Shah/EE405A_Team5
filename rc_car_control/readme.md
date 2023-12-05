# RC Car Control Package

## Overview
This ROS package provides an interface for controlling an RC car using Ackermann steering commands. It subscribes to topics for autonomous mode control, steering, and throttle commands, and translates these into PWM signals for the RC car's motors.

## Dependencies
- ROS (Robot Operating System)
- std_msgs
- ackermann_msgs

## Launching the Node
To launch the RC Car Control node, use the provided launch file:

    `roslaunch rc_car_control rc_car_control.launch`

## Testing script
To run a quick test run the following file: 

    `rosrun rc_car_control test_rc_car_control.py`

## Topics
- `/auto_mode` (std_msgs/Bool): Controls whether the car is in autonomous mode.
- `/auto_cmd/steer` (ackermann_msgs/AckermannDrive): Provides steering angle and steering angle velocity commands.
- `/auto_cmd/throttle` (std_msgs/Int16): Provides throttle commands, represented as speed.

## Configuration
The following parameters can be configured in the launch file:
- `auto_mode_topic`: Topic name for autonomous mode control.
- `steer_topic`: Topic name for steering commands.
- `throttle_topic`: Topic name for throttle commands.