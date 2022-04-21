# Bachelor-E2205
Repository that organizes the work of group E2205

# Controller Package

## Summary

Controller Package ROS2 node with the purpose of taking joystick input and converting to a setpoint change for the PD controller. Joystick input is gathered using Joy library and then converted to body frame actions **surge, sway, heave, roll, pitch, yaw**. These actions are then converted to a setpoint change relative to current state.

The PD controller depends on state estimation information such as position, attitude and velocities. 

## Subscribes to
* sensor_msgs/msg/Joy Joy
* nav_msgs/msg/Odometry State_Estimate (Specific topic name unknown)

## Publishes to
* bluerov_interfaces/msg/ActuatorInput actuation

## PD

Control equation: **tau** = - Kp* **v** - Kd* **z** + **g**

- PD Output: **tau** = [Force_x, Force_y, Force_z, Torque_r, Torque_p, Torque_y]
- Velocities: **v** = [v_x, v_y, v_z, w_r, w_p, w_y]
- Restoring force vector: **g**
- Error Vector: **z** = [x_thilde, y_thilde, z_thilde, sgn(w_thilde)*i_thilde, sgn(w_thilde)*j_thilde, sgn(w_thilde)*k_thilde]
- Position error: **x_tilde** = [x - x_d, y - y_d, z - z_d]
- Attitude error: **q_tilde** = q_d.conjugate() * q



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
