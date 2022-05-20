# Bachelor-E2205
Repository that organizes the work of group E2205

# Controller Package

## Summary

Controller Package ROS2 node with the purpose of taking joystick input and converting to a setpoint change for the PD controller. Joystick input is gathered using Joy library and then converted to body frame actions **surge, sway, heave, roll, pitch, yaw**. These actions are then converted to a setpoint change relative to current state.

The PD controller depends on state estimation information such as position, attitude and velocities. 

Tested on Ubuntu Focal Fossa (20.04) using ROS2 Galactic.

# ROS2

## Subscribes to
* sensor_msgs/msg/Joy Joy (Required for joystick operation)
* nav_msgs/msg/Odometry CSEI/observer/odom (Required for state estimation)
* geometry_msgs/msg/Pose controller/input/setpoint Setpoint sent via ROS2 topic

## Publishes to
* geometry_msgs/msg/Wrench controller/output/desired_forces Force output from controller

## ROS2 Parameters

### Physical properties
| Parameter | Description |
| --- | ----------- |
| Buoancy | Upwards force from buoancy properties (N) |
| Weight | Downwards force from weight properties (N) |
| Centre_of_buoyancy | 3-Dimensional vector [x, y, z] with the centre of buoancy force relative to the ROVs geometrical centre |
| Centre_of_gravity | 3-Dimensional vector [x, y, z] with the centre of gravitational force relative to the ROVs geometrical centre |

### Controller parameters
| Parameter | Description |
| --- | ----------- |
| Proportional_gain_angular | PD Controllers proportional gain which affects rotational control |
| Proportional_gain_linear | PD Controllers proportional gain which affects linear control |
| Integral_gain_angular | PID Controllers integral gain which affects rotational control |
| Integral_gain_linear | PID Controllers integral gain which affects linear control |
| Integral_windup_angular | Maximum windup for angular control |
| Integral_windup_linear | Maximum winduo for linear control |
| Derivative_gain | Derivative gain for the PD controller |

### Operator parameters
| Parameter | Description |
| --- | ----------- |
| Load_setpoint_from_topic | If true, use the setpoint from ROS2 topic |
| Compensate_NED | Compensate joystick input when body frame is in NED |
| Linear_control_xy | Enable/disable control in the xy-plane |
| Linear_control_z | Enable/disable control along the z-axis |
| Enable_controller | Enable/disable controller output values |
| Enable_integrator | Enable/disable integrator in the controller |



# How to use:

If cloning to another folder, remember to change the commands to match the correct folder.

Clone repository:
```
cd ~
mkdir -p bs_ws/src && cd bs_ws/src
git clone https://github.com/ErikRowe/Bachelor-E2205
```

Build and source in terminal.
```
cd ~/bs_ws
colcon build --packages-select controller_package && source install/setup.bash
```
To start the controller node use:
```
ros2 launch controller_package controller_launch.py
```

If using a joystick, start joy in a new terminal window:
```
ros2 run joy joy_node
```

If connecting with Basso, bluerov2_interface_bridge and bluerov_interfaces is required:
```
cd ~/bs_ws
colcon build --packages-select bluerov_interfaces && source install/setup.bash
colcon build --packages-select bluerov2_interface_bridge && source install/setup.bash
```

To start the interface bridge:
```
ros2 run bluerov2_interface_bridge bridge_node
```

Use params.yaml to change node parameters during runtime. Upload changes using:
```
ros2 param load /Control_Node ~/bs_ws/src/Bachelor-E2205/controller_package/params/params.yaml
```
