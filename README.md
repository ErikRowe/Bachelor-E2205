# Bachelor-E2205
Repository that organizes the work of group E2205

# Controller Package

## Summary

Controller Package ROS2 node with the purpose of taking joystick input and converting to a setpoint change for the PD controller. Joystick input is gathered using Joy library and then converted to body frame actions **surge, sway, heave, roll, pitch, yaw**. These actions are then converted to a setpoint change relative to current state.

The PD controller depends on state estimation information such as position, attitude and velocities. 

Tested on Ubuntu Focal Fossa (20.04) using ROS2 Galactic.

# ROS2

## Subscribes to
* sensor_msgs/msg/Joy Joy
* nav_msgs/msg/Odometry CSEI/observer/odom

## Publishes to
* bluerov_interfaces/msg/ActuatorInput actuation

## ROS2 Parameters

### Physical properties
| Parameter | Description |
| --- | ----------- |
| Buoancy | Upwards force from buoancy properties (N) |
| Weight | Upwards force from buoancy properties (N) |
| Centre_of_buoyancy | 3-Dimensional vector [x, y, z] with the centre of buoancy force relative to the ROVs geometrical centre |
| Centre_of_gravity | 3-Dimensional vector [x, y, z] with the centre of gravitational force relative to the ROVs geometrical centre |

### Controller parameters
| Parameter | Description |
| --- | ----------- |
| Proportional_gain_angular | PD Controllers proportional gain which affects rotational control |
| Proportional_gain_linear | PD Controllers proportional gain which affects linear control |
| Derivative_gain | Derivative gain for the PD controller |
| Control_mode | How the control system should behave |
| Control_mode = 0 | Manual control. Joystick input is directly converted to thrust. Requires joystick |
| Control_mode = 1 | PD Control. Joystick input is converted to a setpoint change OR setpoint is provided by ROS2 parameters |

### Operator parameters
| Parameter | Description |
| --- | ----------- |
| Setpoint_input_mode | Which operator input to use |
| Setpoint_input_mode = 0 | Joystick input. Required for manual control |
| Setpoint_input_mode = 1 | ROS2 parameter input. Upload setpoints from setpoint parameter files |
| World_frame_type | How the ROV is represented in the world frame. Affects setpoint changes when operating with joystick |
| World_frame_type = 0 | Expects attitude to have a normal representation (right hand rule) |
| World_frame_type = 1 | Expects attitude to be represented in NED and compensates joystick input to be more intuitive for the operator |

### Setpoints
If Operator_input_mode is set to 1, these values will be used for the PD controller
| Parameter | Description |
| --- | ----------- |
| Attitude_setpoint | Attitude setpoint represented in quaternion representation in the order [w, x, y, z] |
| Position_setpoint | Positional setpoint represented in [x, y, z] |


# How to use:


Clone repository to desired folder:
```
git clone https://github.com/ErikRowe/Bachelor-E2205
```

Build and source in terminal.
```
colcon build && source install/setup.bash
```
To start the controller node use:
```
ros2 launch controller_package controller_launch.py
```

If using a joystick, start joy in a new terminal window:
```
ros2 run joy joy_node
```

Use /your_folder/controller_package/params/ to change node parameters during runtime. Upload changes using:
```
ros2 param load /Control_Node /your_folder/controller_package/params/params.yaml
```
```
ros2 param load /Control_Node /your_folder/controller_package/params/attitude_setpoint.yaml
```
```
ros2 param load /Control_Node /your_folder/controller_package/params/positon_setpoint.yaml
```


## Connect with drone
# Drone side
check .basrc
# Machine side
```
export ROS_DOMAIN_ID=1 or whatever it is on drone side
sudo netplan generate
sudo netplan apply
export ROS_DISCOVERY_SERVER=127.0.0.1:11811 // Unknown if this is absolutely necessary
```

## PD params
```
controll_mode 0 = open loop, 1 = pd controller
Setpoint_input_mode 0 = joystick, 1 = param file
World_frame_type 0 = NED, 1 = Høyrehånds regel
```

## This worked
```
Open new terminal
sudo netplan generate
sudo netplan apply
export ROS_DOMAIN_ID=1
sudo ifconfig lo multicast
```
