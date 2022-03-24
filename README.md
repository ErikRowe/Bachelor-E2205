# Bachelor-E2205
Repository that organizes the work of group E2205

# Controller Package

## Summary

Controller Package ROS2 node with the purpose of requiring joystick input and converting to a setpoint change for the PID. Joystick input is gathered using Joy library and then converted to body frame actions **surge, sway, heave, roll, pitch, yaw**. These actions are then converted to a setpoint change relative to current state.

The PID depends on state estimation information such as position, attitude and velocities. 

## Subscribes to
* sensor_msgs/msg/Joy Joy
* nav_msgs/msg/Odometry State_Estimate (Specific topic name unknown)

## Publishes to
* bluerov_interfaces/msg/ActuatorInput actuation

## PID

Control equation: **tau** = - Kp* **v** - Kd* **z** + **g

- PID Output: **tau** = [Force_x, Force_y, Force_z, Torque_r, Torque_p, Torque_y]
- Velocities: **v** = [v_x, v_y, v_z, w_r, w_p, w_y]
- Restoring force vector: **g**
- Error Vector: **z** = [x_thilde, y_thilde, z_thilde, sgn(w_thilde)*i_thilde, sgn(w_thilde)*j_thilde, sgn(w_thilde)*k_thilde]
- Position error: **x_tilde** = [x - x_d, y - y_d, z - z_d]
- Attitude error: **q_tilde** = q_d.conjugate() * q



